# Create a virtual environment with all tools installed
# ref: https://hub.docker.com/_/alpine
FROM alpine:edge AS env

#############
##  SETUP  ##
#############
ENV PATH=/usr/local/bin:$PATH
RUN apk add --no-cache make

# Install Python
RUN apk add --no-cache python3-dev py3-pip py3-wheel \
 py3-numpy py3-pandas py3-matplotlib py3-scipy
RUN rm -f /usr/lib/python3.*/EXTERNALLY-MANAGED \
&& python3 -m pip install absl-py mypy mypy-protobuf
ENTRYPOINT ["/bin/sh", "-c"]
CMD ["/bin/sh"]

WORKDIR /root
ADD or-tools_amd64_alpine-edge_python_v*.tar.gz .

RUN cd or-tools_*_v* && make test
