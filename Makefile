.ONESHELL:

all:
	@echo '## Make commands ##'
	@echo
	@$(MAKE) -pRrq -f $(lastword $(MAKEFILE_LIST)) : 2>/dev/null | awk -v RS= -F: '/^# File/,/^# Finished Make data base/ {if ($$1 !~ "^[#.]") {print $$1}}' | sort | egrep -v -e '^[^[:alnum:]]' -e '^$@$$' | xargs

build_octomap:
	@mkdir -p src
	@cd src
	@test -d octomap || git clone https://github.com/OctoMap/octomap.git
	@cd octomap
	@git checkout v1.8.0
	@mkdir -p build
	@cd build
	@cmake ..
	@make -j

build_ext: build_octomap
	@python setup.py build_ext -I src/octomap/octomap/include:src/octomap/dynamicEDT3D/include -L src/octomap/lib

install: build_ext
	@python setup.py install
	@echo "\033[1m\nAll is well! You can start using this!\n\n  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(PWD)/src/octomap/lib\n  python -c 'import octomap'\n\033[0m"
