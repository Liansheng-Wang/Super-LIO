# syntax=docker/dockerfile:1.6

ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-perception AS builder

ARG ROS_DISTRO
ARG TARGETARCH
ARG BUILD_JOBS=4
ARG PACKAGE_VERSION=2.0.0
ARG LIVOX_DEB_REPO=Aecric/livox_ros_driver2
ARG LIVOX_DEB_TAG=1.2.6.1
ARG LIVOX_DEB_VERSION=auto
ARG GTSAM_COMMIT=4f66a491ffc83cf092d0d818b11dc35135521612

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    ROS_WS=/opt/super_lio_ws

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=super-lio-apt-cache-${TARGETARCH}-${ROS_DISTRO} \
    --mount=type=cache,target=/var/lib/apt,sharing=locked,id=super-lio-apt-lib-${TARGETARCH}-${ROS_DISTRO} \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        cmake \
        curl \
        g++ \
        libboost-all-dev \
        libeigen3-dev \
        libgoogle-glog-dev \
        libpcl-dev \
        libtbb-dev \
        make \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-${ROS_DISTRO}-pcl-conversions \
        ros-${ROS_DISTRO}-pcl-ros \
        ros-${ROS_DISTRO}-rosidl-default-generators

# Jammy does not provide libgtsam-dev. Build the stable GTSAM 4.2 commit once
# in a cacheable layer for both amd64 local checks and the ARM64 CI runner.
RUN curl -fSL "https://github.com/borglab/gtsam/archive/${GTSAM_COMMIT}.tar.gz" \
        -o /tmp/gtsam.tar.gz && \
    mkdir -p /tmp/gtsam && \
    tar -xzf /tmp/gtsam.tar.gz --strip-components=1 -C /tmp/gtsam && \
    cmake -S /tmp/gtsam -B /tmp/gtsam-build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_PYTHON=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_UNSTABLE=OFF \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON && \
    cmake --build /tmp/gtsam-build --parallel "${BUILD_JOBS}" && \
    cmake --install /tmp/gtsam-build && \
    ldconfig && \
    rm -rf /tmp/gtsam /tmp/gtsam-build /tmp/gtsam.tar.gz

# livox_ros_driver2 is not available from the ROS apt repositories. Resolve the
# release asset by ROS distro and target architecture, as lightning-lm does.
RUN if [ "${TARGETARCH}" = "arm64" ]; then LIVOX_ARCH="arm64"; else LIVOX_ARCH="amd64"; fi && \
    if [ "${LIVOX_DEB_VERSION}" = "auto" ]; then \
        if [ "${LIVOX_DEB_TAG}" = "latest" ]; then \
            RELEASE_API="https://api.github.com/repos/${LIVOX_DEB_REPO}/releases/latest"; \
        else \
            RELEASE_API="https://api.github.com/repos/${LIVOX_DEB_REPO}/releases/tags/${LIVOX_DEB_TAG}"; \
        fi && \
        curl -fSL "${RELEASE_API}" -o /tmp/livox-release.json && \
        LIVOX_DEB_URL="$(python3 -c 'import json,re,sys; ros,arch,path=sys.argv[1:]; assets=json.load(open(path,encoding="utf-8")).get("assets") or []; pattern=re.compile(r"^ros-%s-livox-ros-driver2_.*_%s\.deb$"%(re.escape(ros),re.escape(arch))); matches=[a for a in assets if pattern.match(a.get("name",""))]; print(matches[0]["browser_download_url"]) if matches else (_ for _ in ()).throw(SystemExit("no livox deb asset for ros=%s, arch=%s; assets: %s"%(ros,arch,", ".join(a.get("name","") for a in assets) or "<none>")))' "${ROS_DISTRO}" "${LIVOX_ARCH}" /tmp/livox-release.json)" && \
        rm /tmp/livox-release.json; \
    else \
        LIVOX_DEB_URL="https://github.com/${LIVOX_DEB_REPO}/releases/download/${LIVOX_DEB_TAG}/ros-${ROS_DISTRO}-livox-ros-driver2_${LIVOX_DEB_VERSION}_${LIVOX_ARCH}.deb"; \
    fi && \
    echo "Fetching livox_ros_driver2 from ${LIVOX_DEB_URL}" && \
    curl -fSL "${LIVOX_DEB_URL}" -o /tmp/livox.deb && \
    apt-get install -y /tmp/livox.deb && \
    rm /tmp/livox.deb

WORKDIR ${ROS_WS}
COPY src/ src/

RUN /bin/bash -c "set -eo pipefail; \
    source /opt/ros/${ROS_DISTRO}/setup.bash; \
    colcon build \
        --merge-install \
        --event-handlers console_direct+ \
        --parallel-workers ${BUILD_JOBS} \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON; \
    source install/setup.bash; \
    colcon test \
        --merge-install \
        --event-handlers console_direct+ \
        --parallel-workers ${BUILD_JOBS}; \
    colcon test-result --verbose"

RUN /bin/bash -c "set -euo pipefail; \
    case '${PACKAGE_VERSION}' in \
      [0-9]*) ;; \
      *) echo 'PACKAGE_VERSION must start with a digit' >&2; exit 1 ;; \
    esac; \
    DEB_ARCH=\$(dpkg --print-architecture); \
    DEB_ROOT=/tmp/deb-root; \
    ROS_PREFIX=\${DEB_ROOT}/opt/ros/${ROS_DISTRO}; \
    mkdir -p \${ROS_PREFIX}/lib \${DEB_ROOT}/DEBIAN /output; \
    cp -a install/. \${ROS_PREFIX}/; \
    rm -f \
      \${ROS_PREFIX}/.colcon_install_layout \
      \${ROS_PREFIX}/_local_setup_util_ps1.py \
      \${ROS_PREFIX}/_local_setup_util_sh.py \
      \${ROS_PREFIX}/local_setup.bash \
      \${ROS_PREFIX}/local_setup.ps1 \
      \${ROS_PREFIX}/local_setup.sh \
      \${ROS_PREFIX}/local_setup.zsh \
      \${ROS_PREFIX}/setup.bash \
      \${ROS_PREFIX}/setup.ps1 \
      \${ROS_PREFIX}/setup.sh \
      \${ROS_PREFIX}/setup.zsh; \
    while IFS= read -r file; do \
      sed -i 's|/opt/super_lio_ws/install|/opt/ros/${ROS_DISTRO}|g' \"\${file}\"; \
    done < <(grep -IlR '/opt/super_lio_ws/install' \${ROS_PREFIX}); \
    cp -a /usr/local/lib/libgtsam.so* /usr/local/lib/libmetis-gtsam.so* \${ROS_PREFIX}/lib/; \
    mkdir -p \${ROS_PREFIX}/share/super_lio_loop_backend/gtsam; \
    curl -fSL 'https://raw.githubusercontent.com/borglab/gtsam/${GTSAM_COMMIT}/LICENSE' \
      -o \${ROS_PREFIX}/share/super_lio_loop_backend/gtsam/LICENSE; \
    INSTALLED_SIZE=\$(du -sk \${DEB_ROOT} | cut -f1); \
    printf '%s\n' \
      'Package: ros-${ROS_DISTRO}-super-lio' \
      'Version: ${PACKAGE_VERSION}-0jammy' \
      \"Architecture: \${DEB_ARCH}\" \
      'Maintainer: Super-LIO maintainers <lswang@mail.ecust.edu.cn>' \
      'Depends: ros-${ROS_DISTRO}-rclcpp, ros-${ROS_DISTRO}-pcl-ros, ros-${ROS_DISTRO}-pcl-conversions, ros-${ROS_DISTRO}-rosidl-default-runtime, ros-${ROS_DISTRO}-geometry-msgs, ros-${ROS_DISTRO}-sensor-msgs, ros-${ROS_DISTRO}-std-msgs, ros-${ROS_DISTRO}-visualization-msgs, ros-${ROS_DISTRO}-nav-msgs, ros-${ROS_DISTRO}-livox-ros-driver2, libgoogle-glog0v5, libtbb12, libeigen3-dev, libpcl-dev' \
      \"Installed-Size: \${INSTALLED_SIZE}\" \
      'Section: misc' \
      'Priority: optional' \
      'Description: Super-LIO ROS 2 packages' \
      ' Includes basic, super_lio_loop_msgs, super_lio_loop_backend, and super_lio.' \
      > \${DEB_ROOT}/DEBIAN/control; \
    printf '#!/bin/sh\nset -e\nldconfig\n' > \${DEB_ROOT}/DEBIAN/postinst; \
    printf '#!/bin/sh\nset -e\nldconfig\n' > \${DEB_ROOT}/DEBIAN/postrm; \
    chmod 0755 \${DEB_ROOT}/DEBIAN/postinst \${DEB_ROOT}/DEBIAN/postrm; \
    dpkg-deb --build --root-owner-group \${DEB_ROOT} \
      /output/ros-${ROS_DISTRO}-super-lio_${PACKAGE_VERSION}-0jammy_\${DEB_ARCH}.deb; \
    dpkg-deb --info /output/*.deb; \
    dpkg-deb --contents /output/*.deb | grep -E \
      'packages/(basic|super_lio_loop_msgs|super_lio_loop_backend|super_lio)$'; \
    apt-get install -y /output/*.deb; \
    set +u; \
    source /opt/ros/${ROS_DISTRO}/setup.bash; \
    set -u; \
    for package in basic super_lio_loop_msgs super_lio_loop_backend super_lio; do \
      test \"\$(ros2 pkg prefix \${package})\" = '/opt/ros/${ROS_DISTRO}'; \
    done; \
    if ldd /opt/ros/${ROS_DISTRO}/lib/super_lio/super_lio_node | grep -q 'not found'; then \
      echo 'super_lio_node has unresolved shared libraries' >&2; exit 1; \
    fi; \
    ldd /opt/ros/${ROS_DISTRO}/lib/super_lio_loop_backend/loop_backend_node | \
      grep -q '/opt/ros/${ROS_DISTRO}/lib/libgtsam.so.4'"

FROM scratch AS export
COPY --from=builder /output/*.deb /
