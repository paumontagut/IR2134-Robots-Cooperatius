#!/bin/bash
# Creates symlinks from CamelCase model names to the Fuel cache
# so that Gazebo can resolve model://ModelName URIs

FUEL_CACHE="$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/models"
SYMLINK_DIR="$HOME/rmf-ws/fuel_models"

mkdir -p "$SYMLINK_DIR"

# Map CamelCase names (used in the world file) to lowercase Fuel cache dirs
declare -A MODEL_MAP=(
  ["BathroomSink"]="bathroomsink"
  ["Chair"]="chair"
  ["CleaningSection"]="cleaningsection"
  ["CoffeeTable"]="coffeetable"
  ["Desk"]="desk"
  ["DigitalKiosk"]="digitalkiosk"
  ["Drawer"]="drawer"
  ["ElectronicsRecycling"]="electronicsrecycling"
  ["EscalatorStart"]="escalatorstart"
  ["FemaleVisitor"]="femalevisitor"
  ["FoodCourtTable1"]="foodcourttable1"
  ["Fridge"]="fridge"
  ["HeadphonesRack1"]="headphonesrack1"
  ["InformationCounter"]="informationcounter"
  ["KitchenCountertop"]="kitchencountertop"
  ["KitchenSink"]="kitchensink"
  ["MetalCabinetYellow"]="metalcabinetyellow"
  ["MiRCart"]="mircart"
  ["MonitorAndKeyboard"]="monitorandkeyboard"
  ["MopCart2"]="mopcart2"
  ["MopCart3"]="mopcart3"
  ["MopCart4"]="mopcart4"
  ["NurseTable"]="nursetable"
  ["Oak tree"]="oak tree"
  ["OfficeChairBlack"]="officechairblack"
  ["OfficeChairBlue"]="officechairblue"
  ["OfficeChairGrey"]="officechairgrey"
  ["Pine Tree"]="pine tree"
  ["PotatoChipChair"]="potatochipchair"
  ["RetailKiosk1"]="retailkiosk1"
  ["Sofa"]="sofa"
  ["Table"]="table"
  ["Toilet"]="toilet"
  ["TVStand"]="tvstand"
  ["VendingMachine"]="vendingmachine"
  ["WhiteCabinet"]="whitecabinet"
  ["WoodenChair"]="woodenchair"
)

OK=0
FAIL=0

for camel_name in "${!MODEL_MAP[@]}"; do
  lower_name="${MODEL_MAP[$camel_name]}"
  fuel_dir="$FUEL_CACHE/$lower_name"

  if [ -d "$fuel_dir" ]; then
    # Find the latest version directory
    version_dir=$(find "$fuel_dir" -maxdepth 1 -type d -name "[0-9]*" | sort -V | tail -1)
    if [ -n "$version_dir" ]; then
      # Remove existing symlink/dir if any
      rm -rf "$SYMLINK_DIR/$camel_name"
      ln -s "$version_dir" "$SYMLINK_DIR/$camel_name"
      echo "  OK: $camel_name -> $version_dir"
      OK=$((OK + 1))
    else
      echo "  WARN: $camel_name - no version dir in $fuel_dir"
      FAIL=$((FAIL + 1))
    fi
  else
    echo "  MISSING: $camel_name ($fuel_dir not found)"
    FAIL=$((FAIL + 1))
  fi
done

echo ""
echo "Done: $OK OK, $FAIL failed"
echo ""
echo "Add this to your environment before launching:"
echo "  export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:$SYMLINK_DIR"
