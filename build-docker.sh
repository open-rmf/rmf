#!/bin/bash
set -e

cd "$(dirname $0)"

git clone --depth 1 -b build/rmf https://github.com/open-rmf/rmf_deployment_template.git
pushd rmf_deployment_template
docker build -t ghcr.io/open-rmf/rmf/builder-rmf -f rmf/builder-rmf.Dockerfile .
popd

docker build -t ghcr.io/open-rmf/rmf/rmf_demos - < Dockerfile
