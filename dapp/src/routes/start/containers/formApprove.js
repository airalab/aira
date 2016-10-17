import { bindActionCreators } from 'redux'
import { reduxForm } from 'redux-form'
import _ from 'lodash'
import { submitApprove } from '../../../modules/app/actions';
import Form from '../../../shared/components/app/form';

const validate = (values) => {
  const errors = {};
  if (!values.value) {
    errors.value = 'required'
  } else if (_.toNumber(values.value) > 20) {
    errors.value = 'max 20'
  }
  return errors
};
function mapStateToProps() {
  return {
    fields: ['value'],
    labels: ['How much ETH you want approve for control via Telegram'],
    placeholders: ['0']
  }
}
function mapDispatchToProps(dispatch) {
  return {
    onSubmit: bindActionCreators(submitApprove, dispatch)
  }
}
export default reduxForm({
  form: 'FormApprove',
  validate
}, mapStateToProps, mapDispatchToProps)(Form)
