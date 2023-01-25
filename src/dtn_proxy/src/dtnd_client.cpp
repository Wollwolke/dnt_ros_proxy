#include <dtnd_client.hpp>
#include <stdio.h>

using namespace std;

DtndClient::DtndClient(std::string address, uint16_t port)
{
    http = make_unique<httplib::Client>(address, port);
}

bool DtndClient::registerEndpoint(std::string eid)
{
    auto result = getRequest("/register?" + eid);
    cout << result.content << endl;
    return result.success;
}

DtndClient::Result DtndClient::getRequest(std::string path)
{
    using namespace std::literals::string_literals;
    string content = "";
    if (auto res = http->Get(path))
    {
        if (res->status == 200)
            return Result{true, res->body};
        else
            content = "HTTP return code: "s + to_string(res->status) + " - "s + res->reason;
    }
    else
        content = "HTTP Client Error: "s + to_string(res.error());

    return Result{false, content};
}
