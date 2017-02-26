#!/usr/bin/env python

"""
Script decimates a ground truth file. File should be in the MRPT
GridMapNavSimul format. Use the --help flag for more on the available user
options

"""

import argparse

# Argument Parsing
parser = argparse.ArgumentParser(description='Decimate a ground-truth file by the decimation ratio provided')
parser.add_argument('-i', '--input_gt',
                    default=None,
                    help='Specify the input ground-truth file')
parser.add_argument('-o', '--output_gt',
                    default=None,
                    help='Specify the output file for the decimated ground-truth')
parser.add_argument('-d', '--decimation_ratio',
                    default=2,
                    type=int,
                    help='Specify the decimation ratio. Keep one out of d poses, where d is the decimation ratio')

args = vars(parser.parse_args())

fname_in = args["input_gt"]
fname_out = args["output_gt"]
dec_ratio = args["decimation_ratio"]

print("ground-truth *input* file: {}".format(fname_in))
print("ground-truth *output* file: {}".format(fname_out))
print("Decimate and keep one every *{}* poses".format(dec_ratio))

# parse input file
with open(fname_in, 'r') as f_in:
    lines = f_in.readlines()

# populate output file
with open(fname_out, 'w') as f_out:
    for i, line in enumerate(lines):
        if i % dec_ratio == 0:
            f_out.write(str(line))

print("Decimation finished.")
