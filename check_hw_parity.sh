#!/bin/bash
# Verify the sim still runs the same code, params and map as the real QCar.
#
#   bash check_hw_parity.sh [path/to/hardware/repo]     (default ~/Documents/qcar)
#
# 1. SHARED files must be byte-identical to the hardware repo.
# 2. The generated param files must match the hardware param files.
# 3. ADAPTED hardware files (the sim has a Jazzy port or re-implementation of
#    them) must be unchanged since the port. If one changed, update the sim
#    side by hand, then refresh its hash below.
#
# Exits non-zero on any mismatch.

HW="${1:-$HOME/Documents/qcar}"
SIM="$(cd "$(dirname "$0")" && pwd)"
fail=0

if [ ! -d "$HW/src" ]; then
    echo "Hardware repo not found at $HW"
    exit 2
fi

# sim path | hardware path
# Shared files deliberately changed in the sim first (fixes developed here),
# waiting to be copied to the hardware repo. Reported, but do not fail the
# check. Move an entry back to SHARED once the hardware has the same file.
PENDING_PORT=(
)

SHARED=(
  "src/custom_hybrid_planner/src/hybrid_planner.cpp|src/custom_hybrid_planner/src/hybrid_planner.cpp"
  "src/custom_hybrid_planner/include/custom_hybrid_planner/hybrid_planner.hpp|src/custom_hybrid_planner/include/custom_hybrid_planner/hybrid_planner.hpp"
  "src/custom_hybrid_planner/include/nav2_core/global_planner.hpp|src/custom_hybrid_planner/include/nav2_core/global_planner.hpp"
  "src/rpp_controller/src/regulated_pure_pursuit_controller.cpp|src/rpp_controller/src/regulated_pure_pursuit_controller.cpp"
  "src/rpp_controller/include/rpp_controller/regulated_pure_pursuit_controller.hpp|src/rpp_controller/include/rpp_controller/regulated_pure_pursuit_controller.hpp"
  "src/rpp_controller/include/nav2_core/controller.hpp|src/rpp_controller/include/nav2_core/controller.hpp"
  "src/stanley_controller/src/stanley_controller.cpp|src/stanley_controller/src/stanley_controller.cpp"
  "src/stanley_controller/include/stanley_controller/stanley_controller.hpp|src/stanley_controller/include/stanley_controller/stanley_controller.hpp"
  "src/stanley_controller/include/nav2_core/controller.hpp|src/stanley_controller/include/nav2_core/controller.hpp"
  "src/vector_pursuit_controller/src/vector_pursuit_controller.cpp|src/vector_pursuit_controller/src/vector_pursuit_controller.cpp"
  "src/vector_pursuit_controller/include/vector_pursuit_controller/vector_pursuit_controller.hpp|src/vector_pursuit_controller/include/vector_pursuit_controller/vector_pursuit_controller.hpp"
  "src/vector_pursuit_controller/include/nav2_core/controller.hpp|src/vector_pursuit_controller/include/nav2_core/controller.hpp"
  "src/qcar_nav2_bringup/qcar_nav2_bringup/direct_map_publisher.py|src/qcar_nav2_bringup/qcar_nav2_bringup/direct_map_publisher.py"
  "src/qcar_nav2_bringup/maps/new_map.pgm|maps/new_map.pgm"
)

# hardware path | sha256 at the time of the port (hardware commit 841f794) | sim counterpart
ADAPTED=(
  "src/custom_hybrid_planner/src/hybrid_planner_server.cpp|2d2a28164c84516ba14768b0f6e5e231bfe0c49edd347d438ce18b6cc7c63007|src/custom_hybrid_planner/src/hybrid_planner_server.cpp"
  "src/rpp_controller/src/rpp_controller_server.cpp|952fd118a920b77bfc9ce2486d629ad4babb05df765bfb9e4d7aa5b477e88289|src/rpp_controller/src/rpp_controller_server.cpp"
  "src/stanley_controller/src/stanley_controller_server.cpp|69d279fedcd179f8983109d2e60def172aba766e9108aca00abb345bbe0c5621|src/stanley_controller/src/stanley_controller_server.cpp"
  "src/vector_pursuit_controller/src/vector_pursuit_controller_server.cpp|fc5d016da47656d68919b83614690be3751346135c2cad1bb6f22be2cc421016|src/vector_pursuit_controller/src/vector_pursuit_controller_server.cpp"
  "src/qcar_nav2_bringup/config/bt_navigation.xml|138f2569a4d20b6c220f1dc7b1a4adc0226f8806c3b96977582103321c709fc9|src/qcar_nav2_bringup/config/hw/bt_navigation_{hybrid,navfn}.xml"
  "src/qcar_nav2_bringup/qcar_nav2_bringup/qcar_hardware_interface.py|3384b21fe3f677a703d69984db8381c97a25d0500ecdce754413ee072e43c076|src/qcar_gazebo/scripts/qcar_hw_emulator.py"
  "src/qcar_nav2_bringup/launch/nav2_online.launch.py|cab633561b2048d81ba872fac610942883aed165ce0304ae7ad8824220aadf12|src/qcar_nav2_bringup/launch/navigation.launch.py"
  "maps/new_map.yaml|ff8d83ca587a0f3d2001648b2a1eb38c6122f5401e4e4452f6bd2758c9c6b441|src/qcar_nav2_bringup/maps/new_map.yaml + qcar_gazebo/worlds/qcar_hwmap.sdf"
)

echo "== Shared files (must be identical)"
for entry in "${SHARED[@]}"; do
    IFS='|' read -r sim hw <<< "$entry"
    if cmp -s "$SIM/$sim" "$HW/$hw"; then
        echo "  ok       $sim"
    else
        echo "  DIFFERS  $sim  (vs $HW/$hw)"
        fail=1
    fi
done

echo "== Sim changes pending port to hardware (not a failure)"
for entry in "${PENDING_PORT[@]}"; do
    IFS='|' read -r sim hw <<< "$entry"
    if cmp -s "$SIM/$sim" "$HW/$hw"; then
        echo "  ported   $sim  (identical now -- move it back to SHARED)"
    else
        echo "  PENDING  $sim  ($(diff "$HW/$hw" "$SIM/$sim" | grep -c '^[<>]') lines differ)"
    fi
done

echo "== Params (generated from hardware)"
if python3 "$SIM/src/qcar_nav2_bringup/scripts/sync_hw_params.py" --hw-repo "$HW" --check > /tmp/qcar_parity_params.$$ 2>&1; then
    echo "  ok       config/hw/nav2_params_{rpp,stanley,vector}_{hybrid,navfn}.yaml"
else
    sed 's/^/  /' /tmp/qcar_parity_params.$$
    fail=1
fi
rm -f /tmp/qcar_parity_params.$$

echo "== Adapted hardware files (unchanged since port?)"
for entry in "${ADAPTED[@]}"; do
    IFS='|' read -r hw sum simside <<< "$entry"
    actual=$(sha256sum "$HW/$hw" | cut -d' ' -f1)
    if [ "$actual" = "$sum" ]; then
        echo "  ok       $hw"
    else
        echo "  CHANGED  $hw  -> review $simside, then update its hash here"
        fail=1
    fi
done

if [ $fail -ne 0 ]; then
    echo "PARITY CHECK FAILED"
    exit 1
fi
echo "Parity OK"
