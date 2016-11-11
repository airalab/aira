import React from 'react'
import { connect } from 'react-redux'
import { bindActionCreators } from 'redux'
import { getWeb3, isAccounts } from '../../utils/web3'

import RequiredMist from '../components/app/requiredMist'
import Footer from '../components/app/footer'
import Notification from '../components/app/notification'
import { flashMessage } from '../../modules/app/actions';

import './style.css'

const App = (props) => {
  let content
  if (getWeb3()) {
    if (isAccounts()) {
      content = props.children
    } else {
      content = <p>select account</p>
    }
  } else {
    content = <RequiredMist />
  }

  return (<div>
    <div className="container">
      {content}
    </div>
    <Footer />
    <Notification message={props.flash_message} onClose={() => props.flashMessage('')} />
  </div>)
}

function mapStateToProps(state) {
  return {
    flash_message: state.app.flash_message
  }
}
function mapDispatchToProps(dispatch) {
  const actions = bindActionCreators({
    flashMessage
  }, dispatch)
  return {
    flashMessage: actions.flashMessage
  }
}

export default connect(mapStateToProps, mapDispatchToProps)(App)
