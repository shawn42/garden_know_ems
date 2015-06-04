#!/usr/bin/env ruby

raw = eval(ARGV[0])
puts (raw * 175.72 / (1<<16)) - 46.85
