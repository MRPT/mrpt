/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <gtest/gtest.h>
#include <mrpt/comms/net_utils.h>

#include <cstdint>
#include <string>
#include <vector>

#include "comms_test_server.h"

using namespace mrpt::comms;
using comms_test::OneShotServer;

namespace
{
std::string asText(const std::vector<uint8_t>& v) { return {v.begin(), v.end()}; }

/** URL of the local test server. Note that the port is not part of the URL:
 * it travels in HttpRequestOptions. */
std::string localURL(const std::string& object = "/index.html")
{
  return "http://127.0.0.1" + object;
}
}  // namespace

TEST(net_utils, httpGetRejectsNonHttpURL)
{
  std::vector<uint8_t> content;
  net::HttpRequestOutput out;

  EXPECT_EQ(net::http_get("ftp://example.com/file", content, {}, out), net::http_errorcode::BadURL);
  EXPECT_FALSE(out.errormsg.empty());
  EXPECT_TRUE(content.empty());
}

TEST(net_utils, httpGetRejectsURLWithoutServerName)
{
  std::vector<uint8_t> content;
  net::HttpRequestOutput out;

  EXPECT_EQ(net::http_get("http:///no-server", content, {}, out), net::http_errorcode::BadURL);
  EXPECT_FALSE(out.errormsg.empty());
}

TEST(net_utils, httpGetWithoutOutputStructStillReportsBadURL)
{
  // The optional_ref output is genuinely optional in every early-exit path:
  std::vector<uint8_t> content;
  EXPECT_EQ(net::http_get("not-a-url", content), net::http_errorcode::BadURL);
}

TEST(net_utils, httpGetOnClosedPortFailsToConnect)
{
#ifdef _WIN32
  // CClientTCPSocket::connect() has no Windows implementation of the
  // "wait until the connection attempt completes" step (only Linux and Apple
  // branches exist), so a refused connection is reported as connected there
  // and the following write blocks forever. See the PR discussion.
  GTEST_SKIP() << "connect() does not detect a refused connection on Windows.";
#else
  // Bind a port and immediately release it, so we know nothing is listening:
  unsigned short port = 0;
  {
    auto s = comms_test::listenOnFreePort(port);
    ASSERT_TRUE(s);
  }

  net::HttpRequestOptions opts;
  opts.port = port;
  opts.timeout_ms = 500;

  std::vector<uint8_t> content;
  net::HttpRequestOutput out;
  EXPECT_EQ(net::http_get(localURL(), content, opts, out), net::http_errorcode::CouldntConnect);
  EXPECT_FALSE(out.errormsg.empty());
#endif
}

TEST(net_utils, httpGetOk)
{
  const std::string body = "Hello, world!";
  OneShotServer server(
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: text/plain\r\n"
      "Content-Length: " +
      std::to_string(body.size()) +
      "\r\n"
      "\r\n" +
      body);
  ASSERT_TRUE(server.isReady());

  net::HttpRequestOptions opts;
  opts.port = server.port();
  opts.timeout_ms = 5000;

  std::vector<uint8_t> content;
  net::HttpRequestOutput out;
  EXPECT_EQ(net::http_get(localURL(), content, opts, out), net::http_errorcode::Ok);
  EXPECT_EQ(asText(content), body);
  EXPECT_EQ(out.http_responsecode, 200);
  EXPECT_EQ(out.out_headers["Content-Type"], "text/plain");

  const std::string req = server.waitAndGetRequest();
  EXPECT_NE(req.find("GET /index.html HTTP/1.1\r\n"), std::string::npos);
  EXPECT_NE(req.find("Host: 127.0.0.1\r\n"), std::string::npos);
  // Defaults added by the client:
  EXPECT_NE(req.find("Connection: close\r\n"), std::string::npos);
  EXPECT_NE(req.find("User-Agent: MRPT Library\r\n"), std::string::npos);
}

TEST(net_utils, httpGetAsString)
{
  const std::string body = "text overload";
  OneShotServer server(
      "HTTP/1.1 200 OK\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n" + body);
  ASSERT_TRUE(server.isReady());

  net::HttpRequestOptions opts;
  opts.port = server.port();
  opts.timeout_ms = 5000;

  std::string content;
  EXPECT_EQ(net::http_get(localURL(), content, opts), net::http_errorcode::Ok);
  EXPECT_EQ(content, body);
}

TEST(net_utils, httpGetReportsHttpErrorCodes)
{
  OneShotServer server("HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n");
  ASSERT_TRUE(server.isReady());

  net::HttpRequestOptions opts;
  opts.port = server.port();
  opts.timeout_ms = 5000;

  std::vector<uint8_t> content;
  net::HttpRequestOutput out;
  EXPECT_EQ(net::http_get(localURL(), content, opts, out), net::http_errorcode::OtherHTTPError);
  EXPECT_EQ(out.http_responsecode, 404);
  EXPECT_EQ(out.errormsg, "HTTP error 404");
}

TEST(net_utils, httpGetRejectsNonHttpAnswer)
{
  OneShotServer server("I am not an HTTP server\r\n\r\n");
  ASSERT_TRUE(server.isReady());

  net::HttpRequestOptions opts;
  opts.port = server.port();
  opts.timeout_ms = 5000;

  std::vector<uint8_t> content;
  net::HttpRequestOutput out;
  EXPECT_EQ(net::http_get(localURL(), content, opts, out), net::http_errorcode::OtherHTTPError);
  EXPECT_EQ(out.errormsg, "Server didn't send an HTTP/1.1 answer.");
}

TEST(net_utils, httpGetAcceptsNtripSourcetableAnswer)
{
  // NTRIP casters answer "SOURCETABLE 200 OK" instead of "HTTP/1.1 200 OK":
  const std::string body = "STR;MOUNT;;;;;;;\r\n";
  OneShotServer server(
      "SOURCETABLE 200 OK\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n" + body);
  ASSERT_TRUE(server.isReady());

  net::HttpRequestOptions opts;
  opts.port = server.port();
  opts.timeout_ms = 5000;

  std::vector<uint8_t> content;
  net::HttpRequestOutput out;
  EXPECT_EQ(net::http_get(localURL(), content, opts, out), net::http_errorcode::Ok);
  EXPECT_EQ(out.http_responsecode, 200);
  EXPECT_EQ(asText(content), body);
}

TEST(net_utils, httpGetDecodesChunkedTransferEncoding)
{
  OneShotServer server(
      "HTTP/1.1 200 OK\r\n"
      "Transfer-Encoding: chunked\r\n"
      "\r\n"
      "5\r\nhello\r\n"
      "6\r\n world\r\n"
      "0\r\n\r\n");
  ASSERT_TRUE(server.isReady());

  net::HttpRequestOptions opts;
  opts.port = server.port();
  opts.timeout_ms = 5000;

  std::vector<uint8_t> content;
  net::HttpRequestOutput out;
  EXPECT_EQ(net::http_get(localURL(), content, opts, out), net::http_errorcode::Ok);
  EXPECT_EQ(asText(content), "hello world");
}

TEST(net_utils, httpGetWithoutContentLengthReadsUntilConnectionClose)
{
  const std::string body = "no length header here";
  OneShotServer server("HTTP/1.1 200 OK\r\n\r\n" + body);
  ASSERT_TRUE(server.isReady());

  net::HttpRequestOptions opts;
  opts.port = server.port();
  opts.timeout_ms = 5000;

  std::vector<uint8_t> content;
  EXPECT_EQ(net::http_get(localURL(), content, opts), net::http_errorcode::Ok);
  EXPECT_EQ(asText(content), body);
}

TEST(net_utils, httpGetSendsBasicAuthAndExtraHeaders)
{
  OneShotServer server("HTTP/1.1 200 OK\r\nContent-Length: 0\r\n\r\n");
  ASSERT_TRUE(server.isReady());

  net::HttpRequestOptions opts;
  opts.port = server.port();
  opts.timeout_ms = 5000;
  opts.auth_user = "aladdin";
  opts.auth_pass = "opensesame";
  opts.extra_headers["X-Custom"] = "42";

  std::vector<uint8_t> content;
  EXPECT_EQ(net::http_get(localURL(), content, opts), net::http_errorcode::Ok);

  const std::string req = server.waitAndGetRequest();
  // "aladdin:opensesame" in base64:
  EXPECT_NE(req.find("Authorization: Basic YWxhZGRpbjpvcGVuc2VzYW1l"), std::string::npos);
  EXPECT_NE(req.find("X-Custom: 42\r\n"), std::string::npos);
}

TEST(net_utils, httpRequestPostSendsContentAndLength)
{
  OneShotServer server("HTTP/1.1 200 OK\r\nContent-Length: 0\r\n\r\n");
  ASSERT_TRUE(server.isReady());

  net::HttpRequestOptions opts;
  opts.port = server.port();
  opts.timeout_ms = 5000;

  const std::string payload = "a=1&b=2";
  std::vector<uint8_t> content;
  EXPECT_EQ(
      net::http_request("POST", payload, localURL("/form"), content, opts),
      net::http_errorcode::Ok);

  const std::string req = server.waitAndGetRequest();
  EXPECT_NE(req.find("POST /form HTTP/1.1\r\n"), std::string::npos);
  EXPECT_NE(req.find("Content-Length: 7\r\n"), std::string::npos);
  EXPECT_NE(req.find(payload), std::string::npos);
}

TEST(net_utils, httpRequestUsesRootObjectWhenURLHasNoPath)
{
  OneShotServer server("HTTP/1.1 200 OK\r\nContent-Length: 0\r\n\r\n");
  ASSERT_TRUE(server.isReady());

  net::HttpRequestOptions opts;
  opts.port = server.port();
  opts.timeout_ms = 5000;

  std::vector<uint8_t> content;
  EXPECT_EQ(net::http_get("http://127.0.0.1", content, opts), net::http_errorcode::Ok);

  EXPECT_NE(server.waitAndGetRequest().find("GET / HTTP/1.1\r\n"), std::string::npos);
}

TEST(net_utils, dnsResolveOfNumericAddressIsImmediate)
{
  std::string ip;
  EXPECT_TRUE(net::DNS_resolve_async("127.0.0.1", ip));
  EXPECT_EQ(ip, "127.0.0.1");
}

TEST(net_utils, dnsResolveOfUnknownHostFails)
{
  // ".invalid" is reserved by RFC 2606 and must never resolve:
  std::string ip = "stale";
  EXPECT_FALSE(net::DNS_resolve_async("mrpt-unit-test-host.invalid", ip, 3000));
  EXPECT_TRUE(ip.empty());
}

TEST(net_utils, lastSocketErrorIsDescribed)
{
  // Whatever errno currently holds, this must be a printable description:
  EXPECT_NO_THROW((void)net::getLastSocketErrorStr());
}

TEST(net_utils, pingUnknownHostFails)
{
  std::string output;
  EXPECT_FALSE(net::Ping("mrpt-unit-test-host.invalid", 1, &output));
}
