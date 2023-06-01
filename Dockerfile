# Imagem base do ROS 2 Foxy
FROM ros:iron

# Atualizar pacotes
RUN apt-get update && apt-get install -y \
                      curl \
                      gnupg2 \
                      lsb-release \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*rm

# Crie um diretório de trabalho
WORKDIR /ros2_ws

# Clone o repositório público do GitHub
RUN git clone https://github.com/julianomasson/CEBCitySimulation /ros2_ws/ceb

# Crie um diretório de compilação
WORKDIR /ros2_ws/ceb

# Instale as dependências
RUN apt-get update && rosdep update && rosdep install --from-paths src --ignore-src -r -y

# Compile o código
RUN colcon build --symlink-install

# Configure o ambiente
RUN echo "source /ros2_ws/ceb/install/setup.bash" >> /root/.bashrc

# Set the entry point
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
