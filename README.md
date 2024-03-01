## Using Docker

### Installing Docker

If you haven't installed Docker yet, follow the official documentation to install Docker: [Get Docker](https://docs.docker.com/get-docker/)

### Building Docker Image

To build the Docker image, follow these steps:

```bash
# Build the Docker image in the project root directory
# You can choose any name you prefer for the image
docker build -t huabench/code-llm .
```

This will build a Docker image named `huabench/code-llm` based on the Dockerfile in the current directory.

### Running Docker Container

To run the Docker container, use the following command:

```bash
docker run -d -p 8080:80 huabench/code-llm
```

This will start a container in the background and map port 80 of the container to port 8080 on the host.

### Mapping Your Project into the Container

If you want to map your project directory into the container for development or testing, use the `-v` option:

```bash
docker run -it -p 8080:80 -v /path/to/your/project:/src huabench/code-llm /bin/bash
```

This will map your project directory (`/path/to/your/project`) into the `/src` directory inside the container.

### Using a Pre-built Image

If you prefer to use a pre-built Docker image, simply execute the following command:

```bash
docker pull huabench/code-llm
docker run -d -p 8080:80 huabench/code-llm
```

------

Please note that the options and parameters in the above commands may need to be adjusted based on your project's specifics. Make sure your Dockerfile correctly sets the working directory and port number of your project and that services inside the container can access files on the host when running.