#ifndef SERVICE_API_H
#define SERVICE_API_H


#include <iostream>
#include "grpc++/grpc++.h"
#include "service.grpc.pb.h"

namespace VIO {
namespace Api {
namespace mvins {

using namespace grpc;

class RpcApi
{
public:
  typedef std::shared_ptr<mvins::RpcApi> Ptr;
  explicit RpcApi(std::shared_ptr<Channel> channel);
  ~RpcApi();

  //@RPC_ID=0
  Common::CommandFeedback CheckCommunication();
  Common::CommandFeedback CheckCommunication(const Common::ProtoString& request);
  Common::CommandFeedback CheckCommunication(const Common::ProtoString& request,Common::CommandFeedback& feedback);

  //RPC_ID=1
  Common::CommandFeedback GetSuperPoint(const Common::ProtoString& request, Common::PointCloudXY& response);

private:
  std::unique_ptr<ServiceRpc::Rpc::Stub> stub_;
};

}//mvins
}//Api
}//VIO

#endif // SERVICE_API_H
