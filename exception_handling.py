#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 26 19:21:53 2018

@author: daniellabardini
"""
class Maybe:
    def __init__(self, val, err=None):
        self.val = val
        self.err = err

    def __repr__(self):
        if self.err is not None:
           return 'Maybe('+repr(self.val)+', '+repr(self.err)+')'
        else:
           return 'Maybe('+repr(self.val)+')'

    def do(self, func):  # Bind function
        if self.val is not None:
            try:
                val = func(self.val)
            except Exception as e:
                return Maybe(None, e)
            if not isinstance(val, Maybe):
                return Maybe(val)
            else:
                 return val
        else:
            return Maybe(None, self.err)


##############
##############

class myError(Exception):
    """Base class for exceptions in this module."""
    pass

class myInputError(myError):
    """Exception raised for errors in the input.

    Attributes:
        expression -- input expression in which the error occurred
        message -- explanation of the error
    """

    def __init__(self, expression, message):
        self.expression = expression
        self.message = message