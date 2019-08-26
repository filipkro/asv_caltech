#!/usr/bin/env python

"""ADCP Reader Node

This node for the adcp reader. For implementation details, please go to
adcp_reader/src/adcp_reader.py

Example:
    rosrun adcp_reader adcp_reader.py """

from adcp_reader import adcp_reader

if __name__ == '__main__':
    adcp_reader.main()