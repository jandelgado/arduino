# makefile to build examples with platformi
#
.PHONY: phony

run: phony
	pio run

envdump: phony
	-pio run --target envdump

clean: phony
	-pio run --target clean

upload: phony
	pio run --target upload

monitor: phony
	pio device monitor

tags: phony
	ctags -R

ci:
	find .. -maxdepth 2 -not -path '\.\./\.common/*'  -name Makefile \
		-exec sh -c 'make -C $$(dirname {})' \;
phony:


