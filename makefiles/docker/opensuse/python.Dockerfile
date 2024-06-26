FROM ortools/make:opensuse_swig AS env
RUN zypper refresh \
&& zypper install -y python3-devel python3-pip \
 python3-wheel \
 python3-numpy python3-pandas \
&& zypper clean -a
RUN python3 -m pip install absl-py mypy mypy-protobuf

FROM env AS devel
WORKDIR /home/project
COPY . .

FROM devel AS build
RUN make python

FROM build AS test
RUN make test_python

FROM build AS package
RUN make package_python
