include ../.common/Makefile

all: image1.h image2.h upload

# Convert PNG to RAW 1bpp mono.
# Bit order and color are inverted to match image2cpp / Adafruit drawBitmap format
# (LSB = leftmost pixel, 0 = black).
%.raw: images/%.png
	./conv $< $@

# Convert RAW to C header
%.h: %.raw
	# reverse bit order and write header file
	python3 -c "import sys;d=open('$<','rb').read(); sys.stdout.buffer.write(bytes(int(f'{b:08b}'[::-1],2)^0xff for b in d))" \
	| python3 -c "import sys;n=sys.argv[1]+'_raw';d=sys.stdin.buffer.read();lines=['    '+', '.join(f'0x{b:02x}' for b in d[i:i+12]) for i in range(0,len(d),12)];print(f'const unsigned char {n}[] PROGMEM = {{');print(',\n'.join(lines)+'};')" "$*" > $@


clean-all: clean
	rm -f image.raw image.h
