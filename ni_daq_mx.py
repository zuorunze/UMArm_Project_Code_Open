"""Example of AI raw operation."""

import numpy as np
import nidaqmx
from nidaqmx.constants import AcquisitionType, READ_ALL_AVAILABLE


def ni_daq_mx_subroutine(mp_ft):
    try:
        print("starting ni_daq_mx")
        with nidaqmx.Task() as task:
            in_stream = task.in_stream
            task.ai_channels.add_ai_voltage_chan("Dev1/ai0:13")
            task.timing.cfg_samp_clk_timing(2000.0, sample_mode=AcquisitionType.CONTINUOUS)

            print("N Channel 1 Sample Read Raw: ")

            matrix = [
                [ -0.95369,  -0.62872,  -1.20023, -76.47024,   3.76931,  72.42037],
                [  4.39556,  93.38745,  -1.06242, -44.43761,  -2.38610, -41.28880],
                [ 44.45757,  -2.42077,  45.95316,   0.30684,  42.09624,   0.19023],
                [ -0.16498,  -0.64818,  78.68604,   0.70358, -74.37662,   0.10369],
                [-88.29919,   5.57673,  48.81246,   0.48144,  42.22034,  -0.46392],
                [  2.05728,  50.28905,   1.22358,  46.71367,   2.34030,  46.61061],
            ]
            matrix = np.array(matrix)

            np.set_printoptions(suppress=True, precision=4)

            biasvec = np.zeros(6, dtype=float)
            final_bias  = None

            average_size = 20

            calibrate_size = 120
            calibrate_count = 0

            skip_num = 120
            skip_count = 0

            g = 9.80665

            cal_constant = 35690.0674542

            while True:
                data = in_stream.read(number_of_samples_per_channel=average_size)

                if skip_count < skip_num:
                    skip_count += 1
                    continue


                v0 = 0
                v1 = 0
                v2 = 0
                v3 = 0
                v4 = 0
                v5 = 0

                for i in range(average_size):
                    v0 += (data[i*14+0] - data[i*14+8] ) / (average_size*cal_constant)
                    v1 += (data[i*14+1] - data[i*14+9] ) / (average_size*cal_constant)
                    v2 += (data[i*14+2] - data[i*14+10]) / (average_size*cal_constant)
                    v3 += (data[i*14+3] - data[i*14+11]) / (average_size*cal_constant)
                    v4 += (data[i*14+4] - data[i*14+12]) / (average_size*cal_constant)
                    v5 += (data[i*14+5] - data[i*14+13]) / (average_size*cal_constant)


                stg = np.array([v0, v1, v2, v3, v4, v5])

                if final_bias is None:
                    if calibrate_count < calibrate_size:
                        biasvec += stg
                        calibrate_count += 1
                    else:
                        final_bias = biasvec / calibrate_size
                else:
                    ft_out = matrix @ (stg-final_bias)
                    for i in range(6):
                        mp_ft[i] = ft_out[i]

                    # print(ft_out)


    except Exception as e:
        print("ni_daq_mx_subroutine: ", e)
    return

if __name__ == "__main__":
    with nidaqmx.Task() as task:
        in_stream = task.in_stream
        task.ai_channels.add_ai_voltage_chan("Dev1/ai0:13")
        task.timing.cfg_samp_clk_timing(12000.0, sample_mode=AcquisitionType.CONTINUOUS)

        print("N Channel 1 Sample Read Raw: ")

        count = 0
        matrix = [
            [ -0.95369,  -0.62872,  -1.20023, -76.47024,   3.76931,  72.42037],
            [  4.39556,  93.38745,  -1.06242, -44.43761,  -2.38610, -41.28880],
            [ 44.45757,  -2.42077,  45.95316,   0.30684,  42.09624,   0.19023],
            [ -0.16498,  -0.64818,  78.68604,   0.70358, -74.37662,   0.10369],
            [-88.29919,   5.57673,  48.81246,   0.48144,  42.22034,  -0.46392],
            [  2.05728,  50.28905,   1.22358,  46.71367,   2.34030,  46.61061],
        ]
        matrix = np.array(matrix)

        np.set_printoptions(suppress=True, precision=4)

        biasvec = np.zeros(6, dtype=float)
        final_bias  = None

        average_size = 20

        calibrate_size = 1000
        calibrate_count = 0

        skip_num = 1000
        skip_count = 0

        g = 9.80665

        cal_constant = 35690.0674542

        while True:
            data = in_stream.read(number_of_samples_per_channel=average_size)

            if skip_count < skip_num:
                skip_count += 1
                continue


            v0 = 0
            v1 = 0
            v2 = 0
            v3 = 0
            v4 = 0
            v5 = 0

            for i in range(average_size):
                v0 += (data[i*14+0] - data[i*14+8] ) / (average_size*cal_constant)
                v1 += (data[i*14+1] - data[i*14+9] ) / (average_size*cal_constant)
                v2 += (data[i*14+2] - data[i*14+10]) / (average_size*cal_constant)
                v3 += (data[i*14+3] - data[i*14+11]) / (average_size*cal_constant)
                v4 += (data[i*14+4] - data[i*14+12]) / (average_size*cal_constant)
                v5 += (data[i*14+5] - data[i*14+13]) / (average_size*cal_constant)


            stg = np.array([v0, v1, v2, v3, v4, v5])

            if final_bias is None:
                if calibrate_count < calibrate_size:
                    biasvec += stg
                    calibrate_count += 1
                else:
                    final_bias = biasvec / calibrate_size
            else:
                print(matrix @ (stg-final_bias))

