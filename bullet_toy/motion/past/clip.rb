#!/usr/bin/ruby

s = ARGV[0].to_i
e = ARGV[1].to_i
while true do
	n = STDIN.gets
	puts n
	break if n.chomp == "MOTION"
end

p = STDIN.gets
p = p.split(' ')
puts p[0] + " %d" % (e-s+1) + "\n"
puts STDIN.gets
for i in 1..e
	n = STDIN.gets
	puts n if i >= s
end
