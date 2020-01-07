#!/usr/bin/env python
# -*- coding:utf-8

import abc


class DetectMethod:
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def do_behavior(self):
        raise NotImplementedError()
