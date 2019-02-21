# makefile to build examples with platformi
#
.PHONY: run clean upload monitor ci envdump

run:
	pio run

envdump:
	-pio run --target envdump

clean:
	-pio run --target clean

upload:
	pio run --target upload 

monitor:
	pio device monitor 

tags:
	ctags -R
