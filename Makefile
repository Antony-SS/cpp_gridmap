.PHONY: build configure clean run

configure:
	cmake -B build

build: configure
	cmake --build build --parallel

run: build
	@echo "Usage: make run SCRIPT=<name>"
	./build/scripts/$(SCRIPT)

clean:
	rm -rf build