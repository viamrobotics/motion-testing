BIN_OUTPUT_PATH = bin/$(shell uname -s)-$(shell uname -m)

TOOL_BIN = bin/gotools/$(shell uname -s)-$(shell uname -m)

GIT_REVISION = $(shell git rev-parse HEAD | tr -d '\n')
DATE_COMPILED?=$(shell date +'%Y-%m-%d')
COMMON_LDFLAGS = -X 'go.viam.com/rdk/config.GitRevision=${GIT_REVISION}' -X 'go.viam.com/rdk/config.DateCompiled=${DATE_COMPILED}'
ifdef BUILD_DEBUG
	GCFLAGS = -gcflags "-N -l"
else
	COMMON_LDFLAGS += -s -w
endif
LDFLAGS = -ldflags "-extld=$(shell pwd)/etc/ld_wrapper.sh $(COMMON_LDFLAGS)"

default: static

setup:
	bash etc/setup.sh

GO_FILES=$(shell find . -name "*.go")

GOOS ?= $(shell go env GOOS)
GOARCH ?= $(shell go env GOARCH)

tool-install:
	GOBIN=`pwd`/$(TOOL_BIN) go install \
		github.com/golangci/golangci-lint/cmd/golangci-lint \
		github.com/AlekSi/gocov-xml \
		github.com/axw/gocov/gocov \
		gotest.tools/gotestsum \
		github.com/rhysd/actionlint/cmd/actionlint \
		golang.org/x/tools/cmd/stringer

static:
	rm -f $(BIN_OUTPUT_PATH)/gen_plan
	rm -f $(BIN_OUTPUT_PATH)/plan_motion
	VIAM_STATIC_BUILD=1 GOFLAGS=$(GOFLAGS) go build $(GCFLAGS) $(LDFLAGS) -o $(BIN_OUTPUT_PATH)/gen_plan ./cmd/gen_plan/main.go
	VIAM_STATIC_BUILD=1 GOFLAGS=$(GOFLAGS) go build $(GCFLAGS) $(LDFLAGS) -o $(BIN_OUTPUT_PATH)/plan_motion ./cmd/plan_motion/main.go

clean-all:
	git clean -fxd
# include *.make

