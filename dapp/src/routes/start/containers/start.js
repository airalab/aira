import React, { Component } from 'react'
import { connect } from 'react-redux'
import { bindActionCreators } from 'redux'
import Main from '../components/start/main'
import { load } from '../../../modules/app/actions';

class Container extends Component {
  componentWillMount() {
    this.props.load()
  }
  render() {
    return <Main {...this.props} />
  }
}

function mapStateToProps(state) {
  return {
    balance: state.app.balance,
    approved: state.app.approved
  }
}
function mapDispatchToProps(dispatch) {
  const actions = bindActionCreators({
    load
  }, dispatch)
  return {
    load: actions.load
  }
}

export default connect(mapStateToProps, mapDispatchToProps)(Container)
