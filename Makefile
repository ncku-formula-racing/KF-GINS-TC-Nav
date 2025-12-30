all: format
	@mkdir -p build	
	cmake -S . -B build
	cmake --build build

format:
	@find . \( -path "./thirdparty" -o -path "./build" \) -prune -o \
			-name "*.[ch]" -print \
			| xargs clang-format -i
	@echo "Format!!"

clean:
	rm -rf build
