#!/bin/bash
cd /f/platform/conectric/

rm conectric-version.h

echo '#ifndef CONECTRIC_VERSION_STRING' > conectric-version.h
echo -n '#define CONECTRIC_VERSION_STRING "Contiki-3.x-' >> conectric-version.h
HASH=$(git rev-parse --short HEAD)
echo $HASH\" >> conectric-version.h
echo '#endif'>> conectric-version.h
echo ''
echo '#ifndef CONECTRIC_PROJECT_STRING' >> conectric-version.h
echo -n '#define CONECTRIC_PROJECT_STRING "' >> conectric-version.h
VERSION=$(git describe --abbrev=0 --tags)
echo $VERSION\" >> conectric-version.h
echo '#endif'>> conectric-version.h
echo ''
