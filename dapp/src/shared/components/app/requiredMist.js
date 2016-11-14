import React from 'react'

export default function () {
  return (
    <div className="container">
      <div className="row">
        <div className="col-md-12 text-center">
          <h1>Your browser does not support Web3js</h1>
          <p>download and install one of these add-ons</p>
          <div className="row" style={{ marginTop: 100, marginBottom: 100 }}>
            <div className="col-md-3 col-md-offset-3">
              <a href="https://github.com/ethereum/mist/releases"><img alt="Mist" src="mist.png" style={{ width: 180 }} /></a>
              <h3>Mist</h3>
            </div>
            <div className="col-md-3">
              <a href="https://metamask.io/"><img alt="MetaMask" src="metamask.png" style={{ width: 297 }} /></a>
              <h3>MetaMask</h3>
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}
