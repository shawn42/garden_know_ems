#!/usr/bin/env ruby

raw = eval(ARGV[0])

puts (raw * 125.0 / (1<<16)) - 6
