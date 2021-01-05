#!/usr/bin/env python

from __future__ import print_function, division

import sys
import os
import traceback
from datetime import datetime
import warnings
import math
import csv

import numpy as np
from LabJackPython import Device
import u3


# TODO TODO probably provide cleanup function, so ROS wrapper of stream-to-csv
# script can have this cleanup called when ROS is shutdown?
# (probably need u3 object to be global to achieve this)

# From table 3.2-1 with resolutions and cognate max stream scan frequencies.
# Maximum scan frequencies are in samples/s (shared across all channels).
# https://labjack.com/support/datasheets/u3/operation/stream-mode
resolution_index2max_scan_freq = {
    0: 2500,
    1: 10000,
    2: 20000,
    3: 50000
}
# TODO also provide fn to output resolution / noise (in units of volts, after
# reading Range from some device details via API). noise in table in in units of
# "Counts".

def get_channel_name(device, channel_index):
    """
    Args:
    device (subclass of `LabJackPython.Device`, like `u3.U3`): used to determine
        name and hardwareVersion of device, via `device.deviceName` and
        `device.hardwareVersion`.

    channel_index (int): index of (input) channel to get a name for.

    Returns `str` name of (input) channel, as printed on device case.

    Raises `ValueError` for several types of invalid input.
    """
    if not issubclass(type(device), Device):
        raise ValueError('device must be a subclass of LabJackPython.Device')

    if type(channel_index) is not int:
        raise ValueError('channel_index must be an int')

    if channel_index < 0:
        raise ValueError('channel_index must be positive')

    device_name = device.deviceName
    if device_name != 'U3-HV':
        raise NotImplementedError('only U3-HV currently supported')

    ############################################################################
    # U3-HV
    ############################################################################
    # TODO are these labels the same on all versions of the U3-HV board?
    # (maybe check device.versionInfo (== 18 for me) if not?)
    # (or device.hardwareVersion (== '1.30' for me))
    if device.hardwareVersion != '1.30':
        warnings.warn("get_channel_name only tested for U3-HV with "
            "hardwareVersion=='1.30'. labels on board may be wrong."
        )

    if channel_index <= 3:
        return 'AIN{}'.format(channel_index)
    elif channel_index <= 7:
        return 'FIO{}'.format(channel_index)
    else:
        raise ValueError(('channel index of {} is not a valid input on device '
            '{}').format(channel_index, device_name)
        )
    ############################################################################

    # TODO also support at least the other U3 variants ('U3-LV' at least, maybe
    # also whatever 'U3B' is (see setting of deviceName in u3.py)


# TODO implement callbacks that can be passed into this fn, then pass stuff to
# publish data from ROS wrapper of this script? or should i just relax the
# separation of ROS stuff from this file?
# TODO TODO emulate some kind of triggered acquisition functionality by starting
# a counter on the trigger pin, and then just only start appending to CSV once
# the count is >0 (and also start counting towards duration_s from that point)
# (would need a pin on my stimulus control arduino to go high at the start of
# the stimulus program though...)
def stream_to_csv(csv_path, duration_s=None, input_channels=None,
    resolution_index=0, input_channel_names=None, time_col=True,
    overwrite=False, verbose=False):
    """
    Args:
    csv_path (str): path of CSV to stream data to.

    duration_s (None or float): If `float`, will stop streaming to CSV after
        recording for at least this amount of time (may include slightly more
        samples from last request, as there are currently a fixed number of
        samples returned per request). If `None` (the default), will stream to
        CSV until this process exits.

    input_channels (None or list): A list of `int` channel indexes to acquire
        data from. By default, will be set to [0], which records from just AIN0
        (on a U3-HV).

    resolution_index (int): 0-3, with 0 being the highest resolution, but also
        having the slowest sampling rate. Defaults to 0.

    input_channel_names (dict or None): If passed, must be a dict with a key for
        each `int` in `input_channels`. Maps each channel index to an arbitrary
        `str` name for this channel, which will be used as the name for the
        corresponding column in the CSV. If not passed, the labels of the
        channels on the LabJack hardware will be used.

    time_col (bool): If `True` (the default), a column 'time_s' will be added to
        CSV with time in seconds from beginning of streaming. Not using absolute
        times because the offset between streaming start / stop and when those
        calls are made seems to be hard to predict or measure.

    overwrite (bool): If `False` (default), will raise `IOError` if `csv_path`
        already exists. Otherwise, will overwrite the file.

    verbose (bool): If `True`, will print more output. Defaults to `False`.
    """
    # TODO ros parameter for this (w/ this probably the default).
    # maybe just make settable in ROS wrapper, and just make kwarg here.
    # TODO also allow use of FIO<4-7> inputs? configure as inputs, etc.
    # `int` in [0, 3]. Should correspond to AIN[0-3] labels on U3-HV.
    if input_channels is None:
        input_channels = [0]
    # TODO print string label of each of above channels on particular hardware.
    # maybe assert hardware is HV version, if accessible via API.

    # TODO also thread resolution_index through to ROS param

    if duration_s is not None:
        if duration_s <= 0:
            raise ValueError('duration_s must be positive or None')

    max_scan_freq = resolution_index2max_scan_freq[resolution_index]

    # TODO ok if this isn't divided cleanly by # of channels?
    # (e.g. if # channels == 3). see alternative set of options for
    # <d>.streamConfig (SamplesPerPacket, InternalStreamClockFrequency,
    # DivideClockBy256, ScanInterval)
    # ScanFrequency "sample rate (Hz) = ScanFrequency * NumChannels"
    scan_frequency = max_scan_freq / len(input_channels)

    d = u3.U3()

    # To learn the if the U3 is an HV
    d.configU3()

    # TODO need to actually do some physical calibration procedure first? how to
    # test for that?
    # For applying the proper calibration to readings.
    d.getCalibrationData()

    # TODO necessary? why? also relevant on HV version, which i think has FIO0/1
    # replaced by AIN0/1 which i think can *only* be configured as analog
    # inputs? (which documentation page said this about the HV though...?)
    # Set the FIO0 and FIO1 to Analog (d3 = b00000011)
    # At least with my U3-HV, printing the output of this function is the same
    # whether FIOAnalog=3 or not, with 'FIOAnalog' == 15 in both cases.
    d.configIO(FIOAnalog=3)

    if verbose:
        print("Configuring U3 stream")
    # https://labjack.com/support/datasheets/u3/hardware-description/ain/channel_numbers
    # TODO determine when you'd want the behavior achieved by setting
    # NChannel=32 (see channel_numbers link above)
    single_ended_negative_channel = 31
    n_channels = [single_ended_negative_channel] * len(input_channels)
    # TODO TODO test that ScanFrequency should actually accept the frequency
    # divided by # of channels, and not the raw max scan frequency!!!
    d.streamConfig(NumChannels=len(input_channels), PChannels=input_channels,
        NChannels=n_channels, Resolution=resolution_index,
        ScanFrequency=scan_frequency
    )

    # Both of these device attributes are set in the above `d.streamConfig`
    # call.
    samples_per_request = d.streamSamplesPerPacket * d.packetsPerRequest
    # TODO TODO test whether rhs should be max_scan_freq or scan_frequency
    request_s = samples_per_request * (1 / max_scan_freq)
    if duration_s is not None:
        n_requests = int(math.ceil(duration_s / request_s))

    # Time it takes to sample all the requested input channels.
    all_channel_sample_dt = 1 / scan_frequency

    if verbose:
        print('samples_per_request:', samples_per_request)
        print('samples_per_request / len(input_channels):',
            samples_per_request / len(input_channels)
        )
        print('n_requests:', n_requests)
        print('max_scan_freq:', max_scan_freq)
        print('max_scan_freq / len(input_channels):',
            max_scan_freq / len(input_channels)
        )
        print('duration_s:', duration_s)
        # Since we need the last request to finish, even if it would bring our
        # sample total above the total we are effectively requesting with
        # duration_s.
        actual_duration_s = request_s * n_requests
        print('actual_duration_s:', actual_duration_s)

        print('all_channel_sample_dt:', all_channel_sample_dt)

    channel_names = [get_channel_name(d, i) for i in sorted(input_channels)]

    if input_channel_names is None:
        column_names = channel_names
    else:
        column_names = [input_channel_names[i] for i in input_channels]

    channel2column_names = dict(zip(channel_names, column_names))

    if time_col:
        column_names = ['time_s'] + column_names
        # Starting from 0 within each request. Time offset will be added to this
        # before they are written to CSV rows along with measured data.
        request_times = np.arange(
            # If we start at 0 and use the same step, the stop will be
            # request_s - all_channel_sample_dt.
            start=all_channel_sample_dt,
            stop=(request_s + all_channel_sample_dt),
            step=all_channel_sample_dt
        )
        assert (len(request_times) ==
            int(samples_per_request / len(input_channels)))
        last_time_s = 0.0

    if verbose:
        print('column_names:', column_names)
        print('channel2column_names:', channel2column_names)

    if not overwrite and os.path.exists(csv_path):
        raise IOError(
            'csv_path={} already exists (and overwrite=False)!'.format(
            csv_path
        ))

    # Can't use the 3rd party `future` library `open` to add the `newline`
    # argument for python2, because then `writeheader()` (and probably other
    # write calls) fail with:
    # `TypeError: write() argument 1 must be unicode, not str`
    # csv docs recommend `newline=''` for proper behavior in a few edge cases.
    if sys.version_info >= (3, 0):
        open_kwargs = dict(newline='')
    else:
        open_kwargs = dict()

    with open(csv_path, 'w', **open_kwargs) as csv_file_handle:
        csv_writer = csv.DictWriter(csv_file_handle, fieldnames=column_names)
        csv_writer.writeheader()

        try:
            if verbose:
                print("Start stream")
            d.streamStart()
            start = datetime.now()
            if verbose:
                print("Start time is %s" % start)

            missed = 0
            request_count = 0
            packet_count = 0

            # This calls <d>.processStreamData internally, to apply calibration.
            for r in d.streamData():
                if r is not None:
                    if n_requests is not None and request_count >= n_requests:
                        break

                    if r["errors"] != 0:
                        warnings.warn("Errors counted: {} ; {}".format(
                            r["errors"], datetime.now()
                        ))

                    if r["numPackets"] != d.packetsPerRequest:
                        warnings.warn("----- UNDERFLOW : {} ; {}".format(
                              r["numPackets"], datetime.now()
                        ))

                    if r["missed"] != 0:
                        missed += r['missed']
                        warnings.warn("+++ Missed {}".format(r["missed"]))

                    # TODO only data not used is 'firstPacket':
                    # "The PacketCounter value in the first USB packet."
                    # is this useful? how?

                    # TODO probably warn (first time) if `r` comes back with
                    # more keys than those we expect from `channel_names`
                    # (after removing the keys that are always there from
                    # consideration)
                    # TODO also test that channel_names agree w/ contents of `r`
                    # for the FIO<x> pins on U3-HV

                    row_data_lists = [r[s] for s in channel_names]

                    if time_col:
                        row_data_lists = \
                            [list(request_times + last_time_s)] + row_data_lists
                        last_time_s += request_s

                    row_dicts = [dict(zip(column_names, row))
                        for row in zip(*tuple(row_data_lists))
                    ]
                    # TODO need to take any particular care that shutdown
                    # doesn't happen in the middle of one of these calls?
                    # anything that even could be done?
                    csv_writer.writerows(row_dicts)

                    request_count += 1
                    packet_count += r['numPackets']
                else:
                    # Got no data back from our read.
                    # This only happens if your stream isn't faster than the USB
                    # read timeout, ~1 sec.
                    # TODO should i be warning in this case?
                    print("No data ; {}".format(datetime.now()))

        # TODO what kind of exception is this intended to catch?
        # (and which lines can raise them?)
        except:
            # TODO print to stderr
            print("".join(i for i in traceback.format_exc()))

        # TODO TODO also trigger this cleanup if ROS gets shutdown (to not need
        # to separately specify recording duration here, if specified elsewhere)
        finally:
            stop = datetime.now()
            d.streamStop()
            if verbose:
                print("Stream stopped.\n")
            d.close()

            if verbose:
                sampleTotal = packet_count * d.streamSamplesPerPacket

                scanTotal = sampleTotal / len(input_channels)
                print("%s requests with %s packets per request with %s samples "
                    "per packet = %s samples total." % (request_count,
                    (float(packet_count)/request_count),
                    d.streamSamplesPerPacket, sampleTotal
                ))
                print("%s samples were lost due to errors." % missed)
                sampleTotal -= missed
                print("Adjusted number of samples = %s" % sampleTotal)

                print('sampleTotal * all_channel_sample_dt = ',
                    sampleTotal * all_channel_sample_dt
                )

                runTime = ((stop-start).seconds +
                    float((stop-start).microseconds) / 1000000
                )
                # TODO why does this difference seem to depend on
                # actual_duration_s?  does that mean that the timing between
                # samples is wrong in some places, or just that the startup /
                # shutdown of the stream takes longer in some cases? how to
                # test? maybe measure a reference square wave ~2-4x slower than
                # sample rate?
                print('runTime - actual_duration_s:',
                    runTime - actual_duration_s
                )
                print("The experiment took %s seconds." % runTime)
                print("Actual Scan Rate = %s Hz" % scan_frequency)
                # TODO TODO what causes discrepancies between these and
                # requested scan rate? should i warn if there's enough of a
                # difference? important? any offset between values yielded by
                # streamData, or just at beginning / end?
                print("Timed Scan Rate = %s scans / %s seconds = %s Hz" %
                      (scanTotal, runTime, float(scanTotal)/runTime))
                print("Timed Sample Rate = %s samples / %s seconds = %s Hz" %
                      (sampleTotal, runTime, float(sampleTotal)/runTime))


if __name__ == '__main__':
    #duration_s = None
    duration_s = 600.0
    #input_channels = [0]
    input_channels = [0, 1]
    input_channel_names = ['valve_control', 'pid']
    stream_to_csv('test.csv', duration_s=duration_s,
        input_channels=input_channels, input_channel_names=input_channel_names,
        overwrite=True, verbose=True
    )

