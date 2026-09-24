// Manual composition : le composant AttachServer est instancié à la main dans
// un exécutable, puis ajouté à un executor.
#include <memory>

#include "my_components/attach_server_component.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Multi-thread obligatoire : le callback du service /approach_shelf bloque
  // jusqu'à la fin de la mission pendant que le timer de contrôle, /scan et
  // /odom doivent continuer à être traités.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    2);

  auto attach_server =
      std::make_shared<my_components::AttachServer>(rclcpp::NodeOptions());
  executor.add_node(attach_server);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
