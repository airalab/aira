import React from 'react'
import Identify from './identify'
import Approve from './approve'
import Send from './send'

const Main = props => (
  <div>
    <Identify />
    <br />
    <Approve balance={props.balance} approved={props.approved} />
    <br />
    <Send />
  </div>
)

export default Main
