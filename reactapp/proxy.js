const { createProxyMiddleware } = require("http-proxy-middleware");

module.exports = function () {
  const proxy = createProxyMiddleware({
    target: "http://192.168.16.4/", // API 서버 URL
    maxSockets: 200,
    changeOrigin: true, // 도메인 변경 여부
    ws: true, // Enable WebSocket proxying
  });

  return proxy;
};
