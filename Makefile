all: ova_minimal

ova_minimal: airapkgs/nixos/release.nix
	nix build -f $^ -o $@ $@
