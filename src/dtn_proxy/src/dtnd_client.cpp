#include <stdio.h>

#include <dtnd_client.hpp>
#include <functional>

using std::cout;
using std::endl;
using std::string;

DtndClient::DtndClient(string address, uint16_t port) {
    http = std::make_unique<httplib::Client>(address, port);
    ws = std::make_unique<WsClient>();

    ws->setOpenHandler(std::bind(&DtndClient::onOpen, this));
    ws->setBundleHandler(
        std::bind(&DtndClient::onBundle, this, std::placeholders::_1));
}

void DtndClient::onOpen() {
    ws->send("/json");
    ws->send("/subscribe bla");
}

void DtndClient::onBundle(string bundle) { cout << bundle << endl; }

DtndClient::Result::Result(bool success = false, string content = "")
    : success(success), content(content) {}

bool DtndClient::registerEndpoint(string eid) {
    auto result = getRequest("/register?" + eid);
    cout << result.content << endl;
    ws->connect("ws://localhost:3000/ws");
    return result.success;
}

DtndClient::Result DtndClient::getRequest(string path) {
    using namespace std::literals::string_literals;
    string content = "";
    if (auto res = http->Get(path)) {
        if (res->status == 200)
            return Result(true, res->body);
        else
            content = "HTTP return code: "s + std::to_string(res->status) +
                      " - "s + res->reason;
    } else
        content = "HTTP Client Error: "s + to_string(res.error());

    return Result(false, content);
}
