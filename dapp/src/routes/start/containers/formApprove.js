import { bindActionCreators } from 'redux'
import { reduxForm } from 'redux-form'
import { submitApprove, getApprovedByAddress } from '../../../modules/app/actions';
import Form from '../../../shared/components/app/form';

const validate = (values) => {
  const errors = {};
  if (!values.value) {
    errors.value = 'required'
  }
  return errors
};
function mapStateToProps(state, props) {
  return {
    fields: ['address', 'value'],
    labels: ['Address token contract. You balance: ' + props.balance + ', Approved: ' + props.approved, 'How much tokens you want approve'],
    placeholders: ['Address token contract', '0.1']
  }
}
function mapDispatchToProps(dispatch) {
  return {
    onSubmit: bindActionCreators(submitApprove, dispatch),
    onChangeInput: {
      address: bindActionCreators(getApprovedByAddress, dispatch)
    }
  }
}
export default reduxForm({
  form: 'FormApprove',
  validate
}, mapStateToProps, mapDispatchToProps)(Form)
