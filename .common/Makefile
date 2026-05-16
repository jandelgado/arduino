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

ci: phony
	set -e; \
	find .. -maxdepth 2 \
		-not -path '../.common/*' \
		-name platformio.ini | \
	while read -r ini; do \
		make -C "$$(dirname "$$ini")"; \
	done

phony:


