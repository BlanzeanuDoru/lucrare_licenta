SHELL = /bin/sh

.SUFFIXES: .tar.bz2

ROOT_PATH := .
VARIANTS_PATH := $(ROOT_PATH)/module/variants
RELEASE := release

# Variant list, can be overriden via command line or ENV
VARIANTS?=$(shell ls --hide=*.mk $(VARIANTS_PATH))

ifeq ($(TRAVIS),true)
PRINT_INFO_TRAVIS=print_info_travis
else
PRINT_INFO_TRAVIS=
endif

CORE_VERSION := $(shell grep version= $(ROOT_PATH)/module/platform.txt | sed 's/version=//g')
PACKAGE_NAME := $(basename $(notdir $(CURDIR)))

# -----------------------------------------------------------------------------
# packaging specific
PACKAGE_FOLDER := module

ifeq (postpackaging,$(findstring $(MAKECMDGOALS),postpackaging))
  PACKAGE_FILENAME=$(PACKAGE_NAME)-$(CORE_VERSION).tar.bz2
  PACKAGE_CHKSUM := $(firstword $(shell sha256sum "$(RELEASE)/$(PACKAGE_FILENAME)"))
  PACKAGE_SIZE := $(firstword $(shell wc -c "$(RELEASE)/$(PACKAGE_FILENAME)"))
endif

# end of packaging specific
# -----------------------------------------------------------------------------

.PHONY: all clean print_info postpackaging

# Arduino module packaging:
#   - exclude version control system files, here git files and folders .git, .gitattributes and .gitignore
#   - exclude 'extras' folder
#   - exclude 'obj' folder from variants
all: clean print_info $(PRINT_INFO_TRAVIS)
	@echo ----------------------------------------------------------
	@echo "Packaging module."
	tar --transform "s|module|$(PACKAGE_NAME)-$(CORE_VERSION)|g" --exclude=.gitattributes --exclude=.travis.yml --exclude-vcs --exclude=obj -cjf "$(RELEASE)/$(PACKAGE_NAME)-$(CORE_VERSION).tar.bz2" "$(PACKAGE_FOLDER)"
	$(MAKE) --no-builtin-rules postpackaging -C .
	@echo ----------------------------------------------------------

clean:
	@echo ----------------------------------------------------------
	@echo  Cleanup
	-$(RM) $(RELEASE)/$(PACKAGE_NAME)-*.tar.bz2 $(RELEASE)/package_$(PACKAGE_NAME)_*.json $(RELEASE)/test_package_$(PACKAGE_NAME)_*.json
	@echo ----------------------------------------------------------

print_info:
	@echo ----------------------------------------------------------
	@echo Building $(PACKAGE_NAME) using
	@echo "CURDIR             = $(CURDIR)"
	@echo "OS                 = $(OS)"
	@echo "SHELL              = $(SHELL)"
	@echo "VARIANTS_PATH      = $(VARIANTS_PATH)"
	@echo "VARIANTS           = $(VARIANTS)"
	@echo "OPTIBOOT_VARIANTS  = $(OPTIBOOT_VARIANTS)"
	@echo "CORE_VERSION       = $(CORE_VERSION)"
	@echo "PACKAGE_NAME       = $(PACKAGE_NAME)"

postpackaging:
	@echo "PACKAGE_CHKSUM      = $(PACKAGE_CHKSUM)"
	@echo "PACKAGE_SIZE        = $(PACKAGE_SIZE)"
	@echo "TRAVIS_JOB_NUMBER   = $(TRAVIS_JOB_NUMBER)"
	@echo "TRAVIS_BUILD_NUMBER = $(TRAVIS_BUILD_NUMBER)"
	@echo "PACKAGE_FILENAME    = $(PACKAGE_FILENAME)"
	cat extras/package_index.json.template | sed s/%%VERSION%%/$(CORE_VERSION)/ | sed s/%%FILENAME%%/$(PACKAGE_FILENAME)/ | sed s/%%CHECKSUM%%/$(PACKAGE_CHKSUM)/ | sed s/%%SIZE%%/$(PACKAGE_SIZE)/ > $(RELEASE)/package_$(PACKAGE_NAME)_$(CORE_VERSION)_index.json
	cp $(RELEASE)/package_$(PACKAGE_NAME)_$(CORE_VERSION)_index.json $(RELEASE)/test_package_$(PACKAGE_NAME)_$(CORE_VERSION)_index.json
	@echo "$(RELEASE)/package_$(PACKAGE_NAME)_$(CORE_VERSION)_index.json created"