FROM gcc:11

RUN apt-get update && apt-get install -y \
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

# set working directory
WORKDIR /app

# copy all files from folder onto container
COPY . /app/

RUN mkdir -p /app/users

# compile the code
RUN g++ -o netflix_app netflix.cpp -lcrypto -std=c++17 -lstdc++fs

# run the code
CMD ["./netflix_app"]