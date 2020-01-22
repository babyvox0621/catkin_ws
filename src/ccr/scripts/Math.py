#!/usr/bin/env python
#
# Copyright(C) 2017 Panasonic Corporation. All Right reserved.
# 
# \file        [Math.py]
# \brief       	Utility class for methematics
# \date        2017/02/02  create a new
#
# \note        
#
#
class Math():
	"""Utility class for methematics"""
	@staticmethod
	def roundFloatToDouble(p_floatVal):
		"""cut off unconsiderable value in float value.
           If python read 32bit value from binary data,
           python treats it as double.
           So, unconsidable digit's value is indefine.
           (ex)
           float value in C - float32_t a = 1.0
             -> read after python - 1.000000123456

			This function cuts off it."""
		return round(p_floatVal, 7)




