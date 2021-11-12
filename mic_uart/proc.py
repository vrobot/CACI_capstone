import sys
filename = sys.argv[1]
print(filename)
def twos_complement(hexstr,bits):
	value = int(hexstr,16)
	if value & (1 << (bits-1)):
		value -= 1 << bits
	return value

a = []
b = []
with open(filename, 'r') as f:
	for line in f:
		a.append(line)

for i in a:
	b.append(twos_complement(i, 16))

with open(filename+"_proc.csv", 'w') as f:
	for i in b:
		f.write(str(i) + "\n")

