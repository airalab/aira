import React from 'react'
import { Route, IndexRoute } from 'react-router'
import App from '../shared/containers/app'
import NotFound from '../shared/components/app/notFound'
import { Start } from '../routes/start'

export const routes = () =>
  (<div>
    <Route path="/" component={App}>
      <IndexRoute component={Start} />
    </Route>
    <Route path="*" component={NotFound} />
  </div>)
