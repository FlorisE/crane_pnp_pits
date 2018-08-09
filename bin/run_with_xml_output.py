#! /usr/bin/env python

from rospit.framework import Runner, XMLOutputSettings
from test_picks_and_places import get_test_suite
import logging

xml_output_settings = XMLOutputSettings("pits_test_results.xml")
logger = logging.getLogger("rospit framework")
logger.setLevel(logging.INFO)
ch = logging.StreamHandler()
logger.addHandler(ch)

runner = Runner(logger, xml_output_settings)
runner.run_suite(get_test_suite())
