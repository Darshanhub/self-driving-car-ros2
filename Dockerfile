FROM ros:humble-ros-base

ARG WORKSPACE=/workspace
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

SHELL ["/bin/bash","-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --upgrade pip \
    && pip3 install --no-cache-dir -r /tmp/requirements.txt \
    && rm /tmp/requirements.txt

RUN mkdir -p ${WORKSPACE}/src
WORKDIR ${WORKSPACE}

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
