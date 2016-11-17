import React from 'react'
import FormApprove from '../../containers/formApprove'

const Approve = props => (
  <div>
    <h1>Approve token for @AiraEthBot</h1>
    <FormApprove balance={props.balance} approved={props.approved} />
  </div>
)

export default Approve
