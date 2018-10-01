#!/bin/bash
SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"
mongo < ${SCRIPTPATH}/drop_anchoring_db.js