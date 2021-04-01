// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <inttypes.h>
#include <memory>
#include "turtlesim_controller/srv/harmonic.hpp"
#include "rclcpp/rclcpp.hpp"
#include <math.h>

#define PI 3.14159265358979323846

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class MinimalServer : public rclcpp::Node
{
public:
  MinimalServer()
  : Node("minimal_server")
  {
    service_ = this->create_service<turtlesim_controller::srv::Harmonic>(
      "harmonic", std::bind(&MinimalServer::handle_service, this, _1, _2, _3));
  }

private:

  void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<turtlesim_controller::srv::Harmonic::Request> request,
  const std::shared_ptr<turtlesim_controller::srv::Harmonic::Response> response)
  {
  (void)request_header;
  response->vel = 0.1 + 2*sin(PI*request->pos/7 -2*PI/7);
  }
  rclcpp::Service<turtlesim_controller::srv::Harmonic>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalServer>());
  rclcpp::shutdown();
  return 0;
}
