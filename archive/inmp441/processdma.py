import sys
fn = sys.argv[1]
fn_out = sys.argv[2]

audio_hex = []
tmp = []

def twos_complement(hexstr,bits):
	value = int(hexstr,16)
	if value & (1 << (bits-1)):
		value -= 1 << bits
	return value

i = 0
with open(fn, 'r') as f:
	for line in f:
		for word in line.split(' '):
			if (word != '\n'):
				i += 1
				tmp.append(word.replace('\n',''))
				if not (i%2):
					audio_hex.append(tmp)
					tmp = []


#print(audio_hex)

audio_dec = []

for hex in audio_hex:
	hex_str = hex[1]+hex[0]
	audio_dec.append(twos_complement(hex_str, 16))
	#audio_dec.append(int(hex_str, 16))

#print(audio_dec)

with open(fn_out, 'w') as f:
	for dec in audio_dec:
		f.write(str(dec) + "\n")


