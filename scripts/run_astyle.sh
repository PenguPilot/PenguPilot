#!/bin/sh
find $PENGUPILOT_PATH -type f -name '*.[ch]' | xargs astyle --options=$PENGUPILOT_PATH/scripts/format.astyle

