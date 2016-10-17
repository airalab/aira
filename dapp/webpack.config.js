var path = require('path')
var webpack = require('webpack')
var ExtractTextPlugin = require('extract-text-webpack-plugin')

const config = {
  entry: [
    'babel-polyfill',
    './src/index'
  ],
  output: {
    path: path.join(__dirname, 'public'),
    filename: 'bundle.js',
    publicPath: '/'
  },
  plugins: [
    new webpack.optimize.OccurenceOrderPlugin(),
    new webpack.NoErrorsPlugin(),
    new ExtractTextPlugin('bundle.css')
  ],
  module: {
      preLoaders: [
        {
          test: /\.js$/,
          loaders: ['eslint'],
          exclude: /node_modules/,
          include: [
            path.resolve(__dirname, 'src')
          ]
        }
      ],
      loaders: [
        {
          test: /\.css/,
          loader: ExtractTextPlugin.extract(
              'style-loader',
              'css-loader?modules&sourceMap&importLoaders=1&localIdentName=[local]___[hash:base64:5]'
          )
        },
        {
          loaders: ['react-hot', 'babel-loader'],
          include: [
            path.resolve(__dirname, 'src')
          ],
          test: /\.js$/,
          plugins: ['transform-runtime']
        }
    ],
    noParse: /localforage/
  }
}

if (process.env.NODE_ENV === 'production') {
    config.plugins.push(
        new webpack.optimize.UglifyJsPlugin({
            compress: { warnings: false }
        })
    )
    config.plugins.push(
        new webpack.DefinePlugin({
            'process.env': {
                NODE_ENV: JSON.stringify('production')
            }
        })
    )
} else {
    config.devtool = 'source-map'
    config.entry.push(
        'webpack-hot-middleware/client'
    );
    config.plugins.push(
        new webpack.HotModuleReplacementPlugin()
    );
}

module.exports = config
