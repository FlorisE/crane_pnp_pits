#! /usr/bin/env python

logger = logging.getLogger("rospit framework")
logger.setLevel(logging.INFO)
ch = logging.StreamHandler()
logger.addHandler(ch)
