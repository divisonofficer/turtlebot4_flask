const { merge } = require("webpack-merge");
const common = require("./webpack.common.js");

const { proxy } = require("./proxy");

module.exports = merge(common, {
  mode: "development",
  devtool: "eval",
  devServer: {
    historyApiFallback: true,
    port: 3000,
    hot: true,
    proxy: {
      "/": {
        target: "http://192.168.16.4",
        maxSockets: 1000,
      },
    },
  },
});
