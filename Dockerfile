# parameters
ARG REPO_NAME="CSCI-597F-Candy-Cadets"
ARG DESCRIPTION="Candybot 3000"
ARG MAINTAINER="Wil Apollo Zuber (zuberw@wwu.edu)"
# pick an icon from: https://fontawesome.com/v4.7.0/icons/
ARG ICON="cube"

# ==================================================>
# ==> Do not change the code below this line
ARG ARCH
ARG DISTRO
ARG DOCKER_REGISTRY
ARG BASE_REPOSITORY
ARG BASE_ORGANIZATION=duckietown
ARG BASE_TAG=${DISTRO}-${ARCH}
ARG LAUNCHER=default

# define base image
FROM ${DOCKER_REGISTRY}/${BASE_ORGANIZATION}/${BASE_REPOSITORY}:${BASE_TAG} AS base

# recall all arguments
ARG ARCH
ARG DISTRO
ARG DOCKER_REGISTRY
ARG PROJECT_NAME
ARG PROJECT_DESCRIPTION
ARG PROJECT_MAINTAINER
ARG PROJECT_ICON
ARG PROJECT_FORMAT_VERSION
ARG BASE_TAG
ARG BASE_REPOSITORY
ARG BASE_ORGANIZATION
ARG LAUNCHER
# - buildkit
ARG TARGETPLATFORM
ARG TARGETOS
ARG TARGETARCH
ARG TARGETVARIANT
ARG FSM_NODE_CONTROL="1"

# check build arguments
RUN dt-args-check \
    "PROJECT_NAME" "${PROJECT_NAME}" \
    "PROJECT_DESCRIPTION" "${PROJECT_DESCRIPTION}" \
    "PROJECT_MAINTAINER" "${PROJECT_MAINTAINER}" \
    "PROJECT_ICON" "${PROJECT_ICON}" \
    "PROJECT_FORMAT_VERSION" "${PROJECT_FORMAT_VERSION}" \
    "ARCH" "${ARCH}" \
    "DISTRO" "${DISTRO}" \
    "DOCKER_REGISTRY" "${DOCKER_REGISTRY}" \
    "BASE_REPOSITORY" "${BASE_REPOSITORY}" \
    && dt-check-project-format "${PROJECT_FORMAT_VERSION}"

# define/create repository path
ARG PROJECT_PATH="${SOURCE_DIR}/${PROJECT_NAME}"
ARG PROJECT_LAUNCHERS_PATH="${LAUNCHERS_DIR}/${PROJECT_NAME}"
RUN mkdir -p "${PROJECT_PATH}" "${PROJECT_LAUNCHERS_PATH}"
WORKDIR "${PROJECT_PATH}"

# keep some arguments as environment variables
ENV DT_PROJECT_NAME="${PROJECT_NAME}" \
    DT_PROJECT_DESCRIPTION="${PROJECT_DESCRIPTION}" \
    DT_PROJECT_MAINTAINER="${PROJECT_MAINTAINER}" \
    DT_PROJECT_ICON="${PROJECT_ICON}" \
    DT_PROJECT_PATH="${PROJECT_PATH}" \
    DT_PROJECT_LAUNCHERS_PATH="${PROJECT_LAUNCHERS_PATH}" \
    DT_LAUNCHER="${LAUNCHER}" \
    FSM_NODE_CONTROL="${FSM_NODE_CONTROL}"

# install apt dependencies
COPY ./dependencies-apt.txt "${PROJECT_PATH}/"
RUN dt-apt-install ${PROJECT_PATH}/dependencies-apt.txt

# install python3 dependencies
ARG PIP_INDEX_URL="https://pypi.org/simple"
ENV PIP_INDEX_URL=${PIP_INDEX_URL}
COPY ./dependencies-py3.* "${PROJECT_PATH}/"
RUN dt-pip3-install "${PROJECT_PATH}/dependencies-py3.*"

# copy the source code
COPY ./packages "${PROJECT_PATH}/packages"

# build packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
  catkin build \
    --workspace ${CATKIN_WS_DIR}/

# install launcher scripts
COPY ./launchers/. "${PROJECT_LAUNCHERS_PATH}/"
RUN dt-install-launchers "${PROJECT_LAUNCHERS_PATH}"

# install scripts
COPY ./assets/entrypoint.d "${PROJECT_PATH}/assets/entrypoint.d"
COPY ./assets/environment.d "${PROJECT_PATH}/assets/environment.d"

# define default command
CMD ["bash", "-c", "dt-launcher-${DT_LAUNCHER}"]

# store module metadata
LABEL \
    # module info
    org.duckietown.label.project.name="${PROJECT_NAME}" \
    org.duckietown.label.project.description="${PROJECT_DESCRIPTION}" \
    org.duckietown.label.project.maintainer="${PROJECT_MAINTAINER}" \
    org.duckietown.label.project.icon="${PROJECT_ICON}" \
    org.duckietown.label.project.path="${PROJECT_PATH}" \
    org.duckietown.label.project.launchers.path="${PROJECT_LAUNCHERS_PATH}" \
    # format
    org.duckietown.label.format.version="${PROJECT_FORMAT_VERSION}" \
    # platform info
    org.duckietown.label.platform.os="${TARGETOS}" \
    org.duckietown.label.platform.architecture="${TARGETARCH}" \
    org.duckietown.label.platform.variant="${TARGETVARIANT}" \
    # code info
    org.duckietown.label.code.distro="${DISTRO}" \
    org.duckietown.label.code.launcher="${LAUNCHER}" \
    org.duckietown.label.code.python.registry="${PIP_INDEX_URL}" \
    # base info
    org.duckietown.label.base.organization="${BASE_ORGANIZATION}" \
    org.duckietown.label.base.repository="${BASE_REPOSITORY}" \
    org.duckietown.label.base.tag="${BASE_TAG}"
# <== Do not change the code above this line
# <==================================================

ENV DUCKIETOWN_ROOT="${SOURCE_DIR}"
# used for downloads
ENV DUCKIETOWN_DATA="/tmp/duckietown-data"
RUN echo 'config echo 1' > .compmake.rc

COPY assets/bin/send-fsm-state.sh /usr/local/bin


#   OLD DOCKERFILE
#   =============================================================================================

# parameters
# ARG REPO_NAME="CSCI-597F-Candy-Cadets"
# ARG DESCRIPTION="Candybot 3000"
# ARG MAINTAINER="Wil Apollo Zuber (zuberw@wwu.edu)"
# # pick an icon from: https://fontawesome.com/v4.7.0/icons/
# ARG ICON="cube"

# ==================================================>
# ==> Do not change the code below this line
# ARG ARCH
# ARG DISTRO=ente
# ARG DOCKER_REGISTRY=docker.io
# ARG BASE_IMAGE=dt-ros-commons
# ARG BASE_TAG=${DISTRO}-${ARCH}
# ARG LAUNCHER=default

# define base image
# FROM ${DOCKER_REGISTRY}/duckietown/${BASE_IMAGE}:${BASE_TAG} as base

# # recall all arguments
# ARG DISTRO
# ARG REPO_NAME
# ARG DESCRIPTION
# ARG MAINTAINER
# ARG ICON
# ARG BASE_TAG
# ARG BASE_IMAGE
# ARG LAUNCHER
# # - buildkit
# ARG TARGETPLATFORM
# ARG TARGETOS
# ARG TARGETARCH
# ARG TARGETVARIANT

# # check build arguments
# RUN dt-build-env-check "${REPO_NAME}" "${MAINTAINER}" "${DESCRIPTION}"

# # define/create repository path
# ARG REPO_PATH="${CATKIN_WS_DIR}/src/${REPO_NAME}"
# ARG LAUNCH_PATH="${LAUNCH_DIR}/${REPO_NAME}"
# RUN mkdir -p "${REPO_PATH}" "${LAUNCH_PATH}"
# WORKDIR "${REPO_PATH}"

# # keep some arguments as environment variables
# ENV DT_MODULE_TYPE="${REPO_NAME}" \
#     DT_MODULE_DESCRIPTION="${DESCRIPTION}" \
#     DT_MODULE_ICON="${ICON}" \
#     DT_MAINTAINER="${MAINTAINER}" \
#     DT_REPO_PATH="${REPO_PATH}" \
#     DT_LAUNCH_PATH="${LAUNCH_PATH}" \
#     DT_LAUNCHER="${LAUNCHER}"

# # install apt dependencies
# COPY ./dependencies-apt.txt "${REPO_PATH}/"
# RUN dt-apt-install ${REPO_PATH}/dependencies-apt.txt

# # install python3 dependencies
# ARG PIP_INDEX_URL="https://pypi.org/simple"
# ENV PIP_INDEX_URL=${PIP_INDEX_URL}
# COPY ./dependencies-py3.* "${REPO_PATH}/"
# RUN dt-pip3-install "${REPO_PATH}/dependencies-py3.*"

# # copy the source code
# COPY ./packages "${REPO_PATH}/packages"

# # build packages
# RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
#   catkin build \
#     --workspace ${CATKIN_WS_DIR}/

# # install launcher scripts
# COPY ./launchers/. "${LAUNCH_PATH}/"
# RUN dt-install-launchers "${LAUNCH_PATH}"

# # define default command
# CMD ["bash", "-c", "dt-launcher-${DT_LAUNCHER}"]

# # store module metadata
# LABEL org.duckietown.label.module.type="${REPO_NAME}" \
#     org.duckietown.label.module.description="${DESCRIPTION}" \
#     org.duckietown.label.module.icon="${ICON}" \
#     org.duckietown.label.platform.os="${TARGETOS}" \
#     org.duckietown.label.platform.architecture="${TARGETARCH}" \
#     org.duckietown.label.platform.variant="${TARGETVARIANT}" \
#     org.duckietown.label.code.location="${REPO_PATH}" \
#     org.duckietown.label.code.version.distro="${DISTRO}" \
#     org.duckietown.label.base.image="${BASE_IMAGE}" \
#     org.duckietown.label.base.tag="${BASE_TAG}" \
#     org.duckietown.label.maintainer="${MAINTAINER}"
# # <== Do not change the code above this line
# # <==================================================
