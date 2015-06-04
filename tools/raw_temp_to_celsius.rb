#!/usr/bin/env ruby

raw = eval(ARGV[0])
celsius = (raw * 175.72 / (1<<16)) - 46.85
puts "#{celsius} C"
puts "#{celsius * 9.0/5 + 32} F"
