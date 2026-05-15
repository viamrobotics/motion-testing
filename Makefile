
cli: *.go cmd-cli/*.go
	go build -o cli cmd-cli/*.go

run-gomod: cli
	./cli run gomodversion

run-local: cli
	echo "replace go.viam.com/rdk => ../rdk" >> go.mod
	go mod tidy
	./cli run local
	git checkout go.mod
	go mod tidy
	./cli score gomodversion local

update:
	go get go.viam.com/rdk@latest
	go mod tidy
