#!/bin/bash
# Script to download all Gazebo Fuel models needed by the estce simulation
# These models are from OpenRobotics on fuel.gazebosim.org

set -e

FUEL_CACHE="$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/models"

# List of unique models referenced in estce.world
MODELS=(
  "BathroomSink"
  "Chair"
  "CleaningSection"
  "CoffeeTable"
  "Coke"
  "Construction Cone"
  "Desk"
  "DigitalKiosk"
  "Drawer"
  "ElectronicsRecycling"
  "EscalatorStart"
  "FemaleVisitor"
  "FoodCourtTable1"
  "Fridge"
  "HeadphonesRack1"
  "InformationCounter"
  "KitchenCountertop"
  "KitchenSink"
  "MetalCabinetYellow"
  "MiRCart"
  "MonitorAndKeyboard"
  "MopCart2"
  "MopCart3"
  "MopCart4"
  "NurseTable"
  "Oak tree"
  "OfficeChairBlack"
  "OfficeChairBlue"
  "OfficeChairGrey"
  "Pine Tree"
  "PotatoChipChair"
  "RetailKiosk1"
  "ScrubsTrolley"
  "Sofa"
  "Table"
  "TeleportDispenser"
  "TeleportIngestor"
  "Toilet"
  "TVStand"
  "VendingMachine"
  "WhiteCabinet"
  "WoodenChair"
)

echo "Downloading ${#MODELS[@]} models from Gazebo Fuel..."
echo "============================================================"

FAILED=()
SUCCESS=0

for model in "${MODELS[@]}"; do
  echo ""
  echo ">>> Downloading: $model"

  # Check if model already exists in fuel cache
  MODEL_LOWER=$(echo "$model" | tr '[:upper:]' '[:lower:]')
  if [ -d "$FUEL_CACHE/$MODEL_LOWER" ]; then
    echo "    Already in cache, skipping."
    SUCCESS=$((SUCCESS + 1))
    continue
  fi

  # Download from Fuel using gz fuel download
  if gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/$model" -v 0 2>/dev/null; then
    echo "    OK"
    SUCCESS=$((SUCCESS + 1))
  else
    echo "    FAILED to download"
    FAILED+=("$model")
  fi
done

echo ""
echo "============================================================"
echo "Done! $SUCCESS models downloaded successfully."

if [ ${#FAILED[@]} -gt 0 ]; then
  echo ""
  echo "FAILED models (${#FAILED[@]}):"
  for m in "${FAILED[@]}"; do
    echo "  - $m"
  done
fi

echo ""
echo "Now run setup_fuel_symlinks.sh to create the CamelCase symlinks."
