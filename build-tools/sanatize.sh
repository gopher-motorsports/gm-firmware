# Replace the path here with your path and add it to the pre-build step in each CubeIDE
# DLM script
find . -name "*.mk" -exec sed -i 's/C:\/Users\/hmoca\/Desktop\/gm-firmware/..\/..\/../g' {} +; find . -name "makefile" -exec sed -i 's/C:\\Users\\hmoca\\Desktop\\gm-firmware\\components\\data-logging-module\\/..\\/g' {} +; find . -name "*.d" -exec rm {} +