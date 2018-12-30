all: ova_image

ova_image: airapkgs/nixos/release-aira.nix
	nix build -f $^ -o $@ $@
