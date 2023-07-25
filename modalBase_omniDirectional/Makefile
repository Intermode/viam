DOCKER_CMD = docker run $(DOCKER_SSH_AGENT) $(DOCKER_NETRC_RUN) -v$(HOME)/.ssh:/home/testbot/.ssh:ro -v$(shell pwd):/host --workdir /host --rm -ti $(DOCKER_PLATFORM) ghcr.io/viamrobotics/canon:$(DOCKER_TAG) --testbot-uid $(shell id -u) --testbot-gid $(shell id -g)

ifeq ("Darwin", "$(shell uname -s)")
	# Docker has magic paths for OSX
	DOCKER_SSH_AGENT = -v /run/host-services/ssh-auth.sock:/run/host-services/ssh-auth.sock -e SSH_AUTH_SOCK="/run/host-services/ssh-auth.sock"
else ifneq ("$(SSH_AUTH_SOCK)x", "x")
	DOCKER_SSH_AGENT = -v "$(SSH_AUTH_SOCK):$(SSH_AUTH_SOCK)" -e SSH_AUTH_SOCK="$(SSH_AUTH_SOCK)"
endif

linux-shell: DOCKER_PLATFORM = --platform linux/arm64
linux-shell: DOCKER_TAG = arm64-cache
linux-shell:
	$(DOCKER_CMD) bash
