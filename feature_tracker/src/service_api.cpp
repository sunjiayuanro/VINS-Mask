#include "feature_tracker/service_api.h"

namespace VIO {
namespace Api {
namespace mvins {

RpcApi::RpcApi(std::shared_ptr<Channel> channel)
  : stub_(ServiceRpc::Rpc::NewStub(channel))
{
  std::cout << "rpc version: " << Version() << std::endl;
}

RpcApi::~RpcApi()
{}

//@RPC_ID=0
Common::CommandFeedback RpcApi::CheckCommunication()
{
  Common::ProtoString request;
  Common::CommandFeedback feedback;
  Common::CommandFeedback res;
  ClientContext context;
  Status status = stub_->CheckCommunication(&context,request,&feedback);
  if (status.ok()) {
    res.set_status(Common::StateEnum::SUCCEEDED);
  } else {
    std::cout << "error_code: " << status.error_code() << "\t" << status.error_message()
              << std::endl;
    res.set_status(Common::StateEnum::ABORTED);
  }
  return res;
}

Common::CommandFeedback RpcApi::CheckCommunication(const Common::ProtoString& request)
{
  Common::CommandFeedback feedback;
  Common::CommandFeedback res;
  ClientContext context;
  Status status = stub_->CheckCommunication(&context,request,&feedback);
  if (status.ok()) {
    res.set_status(Common::StateEnum::SUCCEEDED);
  } else {
    std::cout << "error_code: " << status.error_code() << "\t" << status.error_message()
              << std::endl;
    res.set_status(Common::StateEnum::ABORTED);
  }
  return res;
}

Common::CommandFeedback RpcApi::CheckCommunication(const Common::ProtoString& request,Common::CommandFeedback& feedback)
{
  Common::CommandFeedback res;
  ClientContext context;
  Status status = stub_->CheckCommunication(&context,request,&feedback);
  if (status.ok()) {
    res.set_status(Common::StateEnum::SUCCEEDED);
  } else {
    std::cout << "error_code: " << status.error_code() << "\t" << status.error_message()
              << std::endl;
    res.set_status(Common::StateEnum::ABORTED);
  }
  return res;
}

//RPC_ID=1
Common::CommandFeedback RpcApi::GetSuperPoint(const Common::ProtoString& request, Common::PointCloudXY& response)
{
  Common::CommandFeedback res;
  ClientContext context;
  Status status = stub_->GetSuperPoint(&context,request,&response);
  if (status.ok()) {
    res.set_status(Common::StateEnum::SUCCEEDED);
  } else {
    std::cout << "error_code: " << status.error_code() << "\t" << status.error_message()
              << std::endl;
    res.set_status(Common::StateEnum::ABORTED);
  }
  return res;
}

}
}
}
